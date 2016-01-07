package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Techno Team_PC_III on 1/6/2016.
 */
public class ResetArms extends RobotOpMode{

    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        leftPlow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightPlow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        waitForStart();

        while (opModeIsActive()) {

           if(gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1){

               leftPlow.setPower(gamepad1.left_stick_y / 8);
               rightPlow.setPower(gamepad1.left_stick_y / 8);

           }else{

               leftPlow.setPower(0);
               rightPlow.setPower(0);

           }

            waitOneFullHardwareCycle();

        }

    }



}