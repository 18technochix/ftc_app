package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Techno Team_PC_III on 2/2/2016.
 */
public class ResetDispenser extends RobotOpMode {

    public void runOpMode() throws InterruptedException {

        dispL = hardwareMap.dcMotor.get("dL");
        dispR = hardwareMap.dcMotor.get("dR");

        dispL.setDirection(DcMotor.Direction.REVERSE);

        dispL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        dispR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        gyro.startIMU(); //THIS IS TEMPORARY

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad2.left_stick_y > 0.1){

                dispL.setPower(dispPower);
                dispR.setPower(dispPower);

            }else if(gamepad2.left_stick_y < -0.1){

                dispL.setPower(-dispPower);
                dispR.setPower(-dispPower);

            }else{

                dispL.setPower(0);
                dispR.setPower(0);

            }

            waitOneFullHardwareCycle();

        }

    }


}
