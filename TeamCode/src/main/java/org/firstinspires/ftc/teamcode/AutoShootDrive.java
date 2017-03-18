package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 11/13/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Shoot & Drive", group="Autonomous")
//@Disabled
public class AutoShootDrive extends AutoShoot{
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        super.runOpMode();

        robot.moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -50, -50, 4.0, "vortex base");

        while (robot.leftMotor.isBusy()){
            //if (robot.leftMotor.getCurrentPosition() > )
            //how do we know the robot is 80% to its target position
        }
    }
}





