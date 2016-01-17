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
public class QualifierTeleop extends QualifierRobotOpMode{

    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {

           // robot.fL.setPower(0.5);
           teleop();
            waitOneFullHardwareCycle();

        }

    }
    
    // Commands ////////////////////////////////////////////////////////////////////////////////////

    public void teleop() throws InterruptedException {

        // Controls for the wheels

        bL.setPower(-scaleInput(gamepad1.left_stick_y));
        fL.setPower(-scaleInput(gamepad1.left_stick_y));

        bR.setPower(-scaleInput(gamepad1.right_stick_y));
        fR.setPower(-scaleInput(gamepad1.right_stick_y));

        //Controls for the servos
        // The dpad controls the left side, the buttons control the right

        if(gamepad2.dpad_left){ //if left dpad is pressed, move left servo to the outside position

            leftFlipper.setPosition(leftUp);

        }else if(gamepad2.dpad_right){ //if the right dpad is pressed, move left servo to inward position

            leftFlipper.setPosition(leftDown);

        }

        if(gamepad2.b){

            rightFlipper.setPosition(rightUp);

        }else if(gamepad2.x){

            rightFlipper.setPosition(rightDown);

        }

        leftPlow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightPlow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        if(gamepad2.dpad_down){
            leftPlow.setTargetPosition(0);
            leftPlow.setPower(-armPower);
            rightPlow.setTargetPosition(0);
            rightPlow.setPower(-armPower);
        }else if(gamepad2.dpad_up){
            leftPlow.setTargetPosition(armFloor);
            leftPlow.setPower(armPower);
            rightPlow.setTargetPosition(armFloor);
            rightPlow.setPower(armPower);
        }
        else{
            leftPlow.setPower(0.);
            rightPlow.setPower(0.);
        }

        // Plow ////////////////////////////////////////////////////////////////////////////////////

        if(gamepad1.left_bumper){

            plow.setTargetPosition(-80);
            plow.setPower(-plowPower);
            negPlow = true;

        }else if(gamepad1.right_bumper){

            plow.setTargetPosition(80);
            plow.setPower(plowPower);
            negPlow = false;

        }else{

            plow.setTargetPosition(0);
            if(negPlow){
                plow.setPower(plowPower);
            } else{
                plow.setPower(-plowPower);
            }


        }

        if(gamepad1.right_trigger > 0){

            plowTop.setPosition(0.6 *  gamepad1.right_trigger);

        }else{

            plowTop.setPosition(0);

        }

        // Cow Catchers ////////////////////////////////////////////////////////////////////////////

        if(gamepad2.left_trigger > 0 ){

            cowLeft.setPosition(gamepad2.left_trigger);

        } else {

            cowLeft.setPosition(0);

        }

        if(gamepad2.right_trigger > 0) {

            cowRight.setPosition(1 - gamepad2.right_trigger);

        }else{

            cowRight.setPosition(1);

        }


        if(gamepad2.y) {

            cowLeft.setPosition(cowLeftOpen);
            cowRight.setPosition(cowRightOpen);


        }else if(gamepad2.a){

            cowLeft.setPosition(0);
            cowRight.setPosition(1);

        }



    }


}

