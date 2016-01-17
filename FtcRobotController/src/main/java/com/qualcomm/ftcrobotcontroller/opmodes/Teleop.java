//teleop template

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Techno Team_PC_III on 9/27/2015.
 */


// after class, replace "TeleOpTemplate" with your class/"program" name.
public class Teleop extends RobotOpMode{

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

