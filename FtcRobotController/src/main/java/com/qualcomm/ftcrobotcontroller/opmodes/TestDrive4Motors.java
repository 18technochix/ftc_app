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
public class TestDrive4Motors extends LinearOpMode{

    Robot robot = new Robot(true,true);

    final double leftDown = 0.5; // value for left servo's extended position
    final double leftUp = 0.25; //value for left servo's retracted position

    final double rightDown = 0; //value for right servo's extended position
    final double rightUp = 0.25; //value for right servo's retracted position

    public void runOpMode() throws InterruptedException {

        robot.initialize(this);

        waitForStart();

        while (opModeIsActive()) {

           // robot.fL.setPower(0.5);
           robot.teleop(this);

            waitOneFullHardwareCycle();

        }

    }


}

