package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Class for mecanum drive
 * experimenting with Field Relative Drive for ease of steering and aiming
 * Code taken from:https://youtu.be/sFCO4du5IZk?si=7h63JWqiSaWLh6x7
 */
public class MecanumDrive {

    static final double COUNTS_PER_MOTOR_REV = 28;

    static final double GEAR_RATIO = 5.23 * 3.61;

    static final double WHEEL_DIAMETER_INCHES = 75 / 25.4;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);


    static final double STALL_FRACTION = 0.25;
    static final double STALL_GRACE_SECONDS = 0.4;
    static final double STALL_CHECK_SECONDS = 0.15;
    // strafing goes ~10% shorter than the wheels turn, this makes up for it. Tune on the field
    static final double STRAFE_CORRECTION = 1.1;

    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private IMU imu; //built in gyro sensor in ftc

    public void init(HardwareMap hwMap){
        // initializes all motors to their hardware map positions
        // DcMotorEx is the same as DcMotor but lets us set speed in ticks per second
        frontLeft = hwMap.get(DcMotorEx.class, "front_left_motor");
        backLeft = hwMap.get(DcMotorEx.class, "back_left_motor");
        frontRight = hwMap.get(DcMotorEx.class, "front_right_motor");
        backRight = hwMap.get(DcMotorEx.class, "back_right_motor");

        // reverses direction of left motors bc they're on backwards
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Runs motors using encoders
        // used to make movement more accurate (usually for auto, but idk if it would be useful in
        // teleop
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        // sets orientation of robot
        // Based on control hub, the logo of the hub is up and USB is facing forward
        // edit if orientated any other way
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
        resetOrientation();
    }

    public void drive(double forward, double strafe, double rotate){
        double[] power = {
                (forward + strafe + rotate), //front left power
                (forward - strafe + rotate), // back left power
                (forward - strafe - rotate), // front right power
                (forward + strafe - rotate) // back right power
        };

        // loops through power to find greatest magnitude of power
        double max = 0;
        for (double v : power) {

            if (max < Math.abs(v)) {
                max = Math.abs(v);
            }

        }

        // if max speed goes over threshold of 1,
        // we correct the speed by normalizing it
        if (max > 1) {

            for (int i = 0; i < power.length; i++){
                power[i] /= max;
            }

        }

        frontLeft.setPower(power[0]);
        backLeft.setPower(power[1]);
        frontRight.setPower(power[2]);
        backRight.setPower(power[3]);

    }

    // Field relative code
    public void driveFieldRelative(double forward, double strafe, double rotate){
        // obtains angle and distance you want the robot to move
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // obtains robot current angle
        // and rotates based on where robot is currently facing
        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // calculates new forward motion based on angle
        double newForward = r * Math.sin(theta);
        // calculates new strafe based on angle
        double newStrafe = r * Math.cos(theta);

        // calls drive function to move motors based on newly calculated forward, strafe variables
        this.drive(newForward, newStrafe, rotate);
    }

    /*
     * Autonomous: drives straight forward (positive inches) or backward (negative inches)
     * maxWheelRpm caps how fast the WHEELS spin (not the bare motor), so the robot stays safe
     * Stops early if the robot hits something (wheels stop turning even though we're driving)
     * Returns true if it hit something, false if it made it the whole distance
     */
    public boolean driveDistance(double inches, double maxWheelRpm, double timeoutSeconds,
                                 LinearOpMode opMode){
        // all 4 wheels spin the same way to drive straight
        double[] wheelDirections = {1, 1, 1, 1};
        return moveDistance(inches, wheelDirections, maxWheelRpm, timeoutSeconds, opMode);
    }

    /*
     * Autonomous: strafes sideways without turning
     * positive inches = same way as positive strafe in drive(), negative = the other way
     * Same speed cap and hit detection as driveDistance
     */
    public boolean strafeDistance(double inches, double maxWheelRpm, double timeoutSeconds,
                                  LinearOpMode opMode){
        // same wheel pattern as strafing in drive(): front left, back left, front right, back right
        double[] wheelDirections = {1, -1, -1, 1};
        // mecanum rollers slip a little when strafing, so the wheels have to turn a bit more
        return moveDistance(inches * STRAFE_CORRECTION, wheelDirections, maxWheelRpm,
                timeoutSeconds, opMode);
    }

    // shared code for driveDistance and strafeDistance
    private boolean moveDistance(double inches, double[] wheelDirections, double maxWheelRpm,
                                 double timeoutSeconds, LinearOpMode opMode){
        DcMotorEx[] motors = {frontLeft, backLeft, frontRight, backRight};

        int targetTicks = (int) Math.abs(inches * COUNTS_PER_INCH);
        double direction = Math.signum(inches);
        // converts wheel rpm to encoder ticks per second
        double maxTicksPerSecond = maxWheelRpm / 60.0 * COUNTS_PER_MOTOR_REV * GEAR_RATIO;

        // starting positions so we can measure how far we've gone
        int[] start = new int[motors.length];
        for (int i = 0; i < motors.length; i++){
            start[i] = motors[i].getCurrentPosition();
        }

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stallTimer = new ElapsedTime();
        double lastTraveled = 0;
        boolean hitObject = false;

        while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds){
            // average distance traveled by all 4 wheels
            double traveled = 0;
            for (int i = 0; i < motors.length; i++){
                traveled += Math.abs(motors[i].getCurrentPosition() - start[i]);
            }
            traveled /= motors.length;

            if (traveled >= targetTicks){
                break;
            }

            // slows down over the last 6 inches so we don't overshoot
            double remainingInches = (targetTicks - traveled) / COUNTS_PER_INCH;
            double speed = maxTicksPerSecond * Math.max(0.3, Math.min(1.0, remainingInches / 6.0));

            for (int i = 0; i < motors.length; i++){
                motors[i].setVelocity(direction * wheelDirections[i] * speed);
            }


            if (stallTimer.seconds() >= STALL_CHECK_SECONDS){
                double expected = speed * stallTimer.seconds();
                if (timer.seconds() > STALL_GRACE_SECONDS
                        && (traveled - lastTraveled) < expected * STALL_FRACTION){
                    hitObject = true;
                    break;
                }
                lastTraveled = traveled;
                stallTimer.reset();
            }
        }

        // stops all motors
        for (DcMotorEx motor : motors){
            motor.setVelocity(0);
        }

        return hitObject;
    }

    // resets orientation to avoid drift
    public void resetOrientation(){

        imu.resetYaw();

    }

    // returns the angle the robot is facing
    public double returnAngle(){
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // normalizes the angle (imu measures angles over 360 and under 0
        angle = angle % 360;
        if (angle < 0){

            angle += 360;

        }

        if (angle >= 359.95){
            angle = 0;
        }

        return angle;

    }
}
