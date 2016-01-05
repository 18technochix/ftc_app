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

        if(gamepad2.dpad_down){
            leftPlow.setTargetPosition(0);
            leftPlow.setPower(cowPower);
            rightPlow.setTargetPosition(0);
            rightPlow.setPower(cowPower);
        }else if(gamepad2.dpad_up){
            leftPlow.setTargetPosition(500);
            leftPlow.setPower(cowPower);
            rightPlow.setTargetPosition(500);
            rightPlow.setPower(cowPower);
        }
        else{
            leftPlow.setPower(0.);
            rightPlow.setPower(0.);
        }

        /*
        if(gamepad2.a){

        }else if(gamepad2.y){

        }
        else{

        }
        */


    }


}

