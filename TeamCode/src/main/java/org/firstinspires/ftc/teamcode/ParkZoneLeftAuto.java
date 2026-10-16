package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * LEAVE + PARK when the loading zone is on the robot's LEFT at the start
 * All the real code is in ParkAuto
 */
@Autonomous(name="Park - Zone on Left", group="buzz")
public class ParkZoneLeftAuto extends ParkAuto {
    @Override
    boolean zoneIsOnLeft(){
        return true;
    }
}
