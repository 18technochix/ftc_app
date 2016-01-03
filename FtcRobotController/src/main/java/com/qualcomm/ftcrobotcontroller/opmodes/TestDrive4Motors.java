//teleop template

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Techno Team_PC_III on 9/27/2015.
 */


// after class, replace "TeleOpTemplate" with your class/"program" name.
public class TestDrive4Motors extends RobotOpMode{

    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {

           // robot.fL.setPower(0.5);
           teleop();
            waitOneFullHardwareCycle();

        }

    }


}

