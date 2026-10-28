package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

/*
 * SHOOT + LEAVE + PARK auto
 * 1. start in front of our hive's non-tipped cell, facing it
 * 2. shoot (not built yet, just waits for now)
 * 3. strafe LEFT until we're almost touching our alliance wall
 * 4. drive FORWARD until the front of the robot is just inside the loading zone (PARK, 5 pts)
 *
 * FIELD MAP (red alliance, looking down from above, audience at the bottom)
 * x = inches from the RED wall, y = inches from the AUDIENCE wall, field is 144 x 144
 *
 *   y=144 +----------------------------------+
 *         |                                  |
 *   y=120 |####  <- red loading zone          |
 *         |####     x 0..11, y ~96..119       |
 *    y=96 |####                               |
 *         |        [red cell]   [blue]        |   hive frame: x ~47..97, y ~52..91
 *         |          (hive)                   |   red cells: x ~47..71, one at y ~55, one at y ~90
 *         |*       [red cell]   [blue]        |   * = flower on red wall, y ~48
 *         |           ^                       |
 *         |         ROBOT  (starts here,      |
 *         |                 facing +y)        |
 *     y=0 +----------------------------------+
 *        x=0 (red wall)                  x=144
 *
 * Numbers come from Figure 9-2 of the game manual (measured off the picture), so they are
 * close but not exact. Blue is the same field turned around, so this auto works for both alliances
 */
@Autonomous(name="Hive Shoot + Park", group="buzz")
public class HiveShootParkAuto extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();

    // robot size in inches (18 x 18 is the max size, MEASURE OURS)
    static final double ROBOT_LENGTH = 18;  // front to back
    static final double ROBOT_WIDTH = 18;   // side to side

    // where the CENTER of the robot starts
    // x: in line with the middle of the red cells (~47..71)
    static final double START_X = 59;
    // y: robot back 21" off the audience wall, front ~13" from the hive frame
    static final double START_Y = 30;

    // gap between the robot's left side and the wall after strafing.
    // not 0 so we don't scrape the wall, and big enough to clear the flower on the wall at y ~48
    static final double WALL_GAP = 6;
    // where the loading zone starts (y), and how far the robot front goes past that line
    static final double ZONE_START_Y = 96;
    static final double ZONE_OVERLAP = 3;

    // how long to sit still and shoot before moving (milliseconds, 5000 = 5 seconds)
    static final long SHOOT_WAIT_MS = 5000;

    // speed cap for the wheels in RPM (top speed is ~300). 200 is ~2/3 speed, ~30 inches/sec.
    // not 300 because the motors need some extra power left over to hold the speed steady
    // when the battery gets low. Still slows down over the last 6" of each move to stop accurately
    static final double SAFE_WHEEL_RPM = 200;
    static final double TIMEOUT_SECONDS = 8;

    @Override
    public void runOpMode(){
        drive.init(hardwareMap);

        // how far to strafe left: from the start x to (wall gap + half the robot)
        double strafeLeftInches = START_X - (WALL_GAP + ROBOT_WIDTH / 2);
        // how far forward: from the robot's front edge to just past the zone line
        double forwardInches = (ZONE_START_Y + ZONE_OVERLAP) - (START_Y + ROBOT_LENGTH / 2);

        telemetry.addData("Strafe left", "%.1f in", strafeLeftInches);
        telemetry.addData("Forward", "%.1f in", forwardInches);
        telemetry.addData("Status", "Ready, press Play");
        telemetry.update();

        waitForStart();

        // 1. shoot. TODO: outtake code goes here
        // waits long enough to shoot all 4 preloaded balls, even with a slow outtake
        sleep(SHOOT_WAIT_MS);

        // 2. strafe left toward our wall (negative = left)
        boolean hitStrafe = drive.strafeDistance(-strafeLeftInches,
                SAFE_WHEEL_RPM, TIMEOUT_SECONDS, this);

        sleep(250);

        // 3. forward along the wall until we're just in the loading zone
        boolean hitForward = drive.driveDistance(forwardInches,
                SAFE_WHEEL_RPM, TIMEOUT_SECONDS, this);

        telemetry.addData("Strafe", hitStrafe ? "Hit something, stopped early" : "Done");
        telemetry.addData("Forward", hitForward ? "Hit something, stopped early" : "Done");
        telemetry.update();

        // keeps telemetry on screen until auto ends
        while (opModeIsActive()){
            idle();
        }
    }
}
