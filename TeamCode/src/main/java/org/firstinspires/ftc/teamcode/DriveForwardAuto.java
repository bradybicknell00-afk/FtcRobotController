package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/*
 * Simple autonomous: drives forward, then backward
 * Stops early if the robot runs into something
 * LinearOpMode runs top to bottom like a script, so it's easier for auto
 */
@Autonomous(name="Drive Forward Auto", group="buzz")
public class DriveForwardAuto extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();

    // how far to drive (inches). Negative = backward
    static final double FORWARD_INCHES = 24;
    static final double BACKWARD_INCHES = -12;
    // speed cap for the wheels in RPM. Start low and raise it once it works
    static final double SAFE_WHEEL_RPM = 100;
    // max seconds each move can take before giving up
    static final double TIMEOUT_SECONDS = 5;

    @Override
    public void runOpMode(){
        drive.init(hardwareMap);

        telemetry.addData("Status", "Ready, press Play");
        telemetry.update();

        // waits for the play button on the driver station
        waitForStart();

        boolean hitForward = drive.driveDistance(FORWARD_INCHES, SAFE_WHEEL_RPM, TIMEOUT_SECONDS, this);
        telemetry.addData("Forward", hitForward ? "Hit something, stopped early" : "Done");
        telemetry.update();

        sleep(500);

        boolean hitBackward = drive.driveDistance(BACKWARD_INCHES, SAFE_WHEEL_RPM, TIMEOUT_SECONDS, this);
        telemetry.addData("Forward", hitForward ? "Hit something, stopped early" : "Done");
        telemetry.addData("Backward", hitBackward ? "Hit something, stopped early" : "Done");
        telemetry.update();

        // keeps telemetry on screen until auto ends
        while (opModeIsActive()){
            idle();
        }
    }
}
