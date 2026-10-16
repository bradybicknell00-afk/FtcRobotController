package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * LEAVE + PARK when the loading zone is on the robot's RIGHT at the start
 * All the real code is in ParkAuto
 */
@Autonomous(name="Park - Zone on Right", group="buzz")
public class ParkZoneRightAuto extends ParkAuto {
    @Override
    boolean zoneIsOnLeft(){
        return false;
    }
}
