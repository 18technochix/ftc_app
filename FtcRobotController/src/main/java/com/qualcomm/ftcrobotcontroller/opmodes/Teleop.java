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

        gyro.startIMU(); //THIS IS TEMPORARY

        waitForStart();

        while (opModeIsActive()) {
            teleop();
            waitOneFullHardwareCycle();

        }

    }
    
    // Commands ////////////////////////////////////////////////////////////////////////////////////

    public void teleop() throws InterruptedException {

        // Wheels //////////////////////////////////////////////////////////////////////////////////

        bL.setPower(scaleInput(gamepad1.left_stick_y));
        fL.setPower(scaleInput(gamepad1.left_stick_y));

        bR.setPower(scaleInput(gamepad1.right_stick_y));
        fR.setPower(scaleInput(gamepad1.right_stick_y));

        // TEMP GYRO ------------------------------------------------------------------------------

        /*

            This returns the different values of the IMU unit to show how far we have moved. It's
            printing back to the Driver Station.

         */

        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
        telemetry.addData("Pitches: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));


        // Cow Catchers ////////////////////////////////////////////////////////////////////////////

        /*
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
        */

        // Dispenser Arms //////////////////////////////////////////////////////////////////////////

        /*
            The dispenser arms are controlled by two motors that are run with encoders.

            On the second gamepad, the left joystick's y values (up and down) control wether the
            arms move (in sync) forward or backwards until it reaches a set encoder count.
            dispPosition is the lowest/farthest out we want the dispenser to go (which by a
            matter of fact is the high goal on the ramp) and the other position is 0, which is
            naturally selected as the resting position of the motors when the program starts
            (wherever the motors are when the program is started is considered encoder position
            zero).

            This allows us to use the motors as standard (not continuous) servos with much more
            power and the ability to use any angle range we want. We measure the 'angle' in encoder
            counts.

            The value 0.1 in the if statements is a fail-safe number for the joysticks. In the past
            we have had sticky joysticks that even though they are centered, the values still read
            back as if they are moved just the slightest bit. This just ensures that we want the
            arms to move only when the joystick is moved by a driver.

            dispPower is a constant in our RobotOpMode class that is used to specify how fast we
            want the dispenser arms to travel. Having it there makes it easier to change the value
            in all of our TeleOp and Autonomous programs on a global scale.
         */

        dispL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        dispR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        if(gamepad2.left_stick_y > 0.1){

            dispL.setTargetPosition(dispPosition);
            dispR.setTargetPosition(dispPosition);

            dispL.setPower(dispPower);
            dispR.setPower(dispPower);

        }else if(gamepad2.left_stick_y < -0.1){

            dispL.setTargetPosition(0);
            dispR.setTargetPosition(0);

            dispL.setPower(-dispPower);
            dispR.setPower(-dispPower);

        }else{

            dispL.setPower(0);
            dispR.setPower(0);

        }


    }// end of teleop()



}

