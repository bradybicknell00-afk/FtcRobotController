package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/*
 * Shared code for the LEAVE + PARK autos (ParkZoneLeftAuto and ParkZoneRightAuto)
 * Not an OpMode by itself, the two autos use this and only pick the strafe direction
 *
 * Setup: robot touching the alliance wall, facing the hive, right next to (NOT in) the LOADING ZONE
 * 1. drive forward off the wall      -> LEAVE (3 pts)
 * 2. strafe sideways toward the zone -> PARK  (5 pts), partly inside the 11" deep zone
 * Works for red and blue, the field is the same rotated around
 */
public abstract class ParkAuto extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();

    // inches off the wall. Must stay under ~11" so the robot still overlaps the zone
    static final double FORWARD_INCHES = 5;
    // inches sideways into the zone (zone is 23" long along the wall)
    static final double STRAFE_INCHES = 15;
    // if the robot drives backward instead of forward, change to -1
    static final double FORWARD_SIGN = 1;
    // if "Zone on Left" strafes RIGHT instead, change to 1
    static final double LEFT_STRAFE_SIGN = -1;

    // speed cap for the wheels in RPM (top speed is ~300)
    static final double SAFE_WHEEL_RPM = 100;
    static final double TIMEOUT_SECONDS = 5;

    // each auto says which side the loading zone is on (from the robot's point of view)
    abstract boolean zoneIsOnLeft();

    @Override
    public void runOpMode(){
        drive.init(hardwareMap);

        telemetry.addData("Zone side", zoneIsOnLeft() ? "LEFT" : "RIGHT");
        telemetry.addData("Status", "Ready, press Play");
        telemetry.update();

        waitForStart();

        // 1. off the wall for LEAVE
        boolean hitForward = drive.driveDistance(FORWARD_SIGN * FORWARD_INCHES,
                SAFE_WHEEL_RPM, TIMEOUT_SECONDS, this);

        sleep(250);

        // 2. sideways into the loading zone for PARK
        double strafeSign = zoneIsOnLeft() ? LEFT_STRAFE_SIGN : -LEFT_STRAFE_SIGN;
        boolean hitStrafe = drive.strafeDistance(strafeSign * STRAFE_INCHES,
                SAFE_WHEEL_RPM, TIMEOUT_SECONDS, this);

        telemetry.addData("Forward", hitForward ? "Hit something, stopped early" : "Done");
        telemetry.addData("Strafe", hitStrafe ? "Hit something, stopped early" : "Done");
        telemetry.update();

        // keeps telemetry on screen until auto ends
        while (opModeIsActive()){
            idle();
        }
    }
}
