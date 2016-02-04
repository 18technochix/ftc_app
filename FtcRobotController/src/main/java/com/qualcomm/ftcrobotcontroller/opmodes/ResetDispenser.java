package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Techno Team_PC_III on 2/2/2016.
 */
public class ResetDispenser extends RobotOpMode {

  //  @Override
    public void runOpMode() throws InterruptedException {

        dispL = hardwareMap.dcMotor.get("dL");
        dispR = hardwareMap.dcMotor.get("dR");

        dispR.setDirection(DcMotor.Direction.REVERSE);

        runDispenserWithoutEncoders();

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
