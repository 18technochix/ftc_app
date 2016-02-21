//teleop template

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;

import static java.lang.Math.abs;

/**
 * Created by Techno Team_PC_III on 9/27/2015.
 */


// after class, replace "TeleOpTemplate" with your class/"program" name.
public class Teleop extends RobotOpMode{

    /*

        The value 0.1 in the if statements is a fail-safe number for the joysticks. In the past
            we have had sticky joysticks that even though they are centered, the values still read
            back as if they are moved just the slightest bit. This just ensures that we want the
            arms to move only when the joystick is moved by a driver.

     */

    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        harvester.setPower(0);
        waitOneFullHardwareCycle();

       // gyro.startIMU(); //THIS IS TEMPORARY

        waitForStart();

        while (opModeIsActive()) {
            teleop();
            waitOneFullHardwareCycle();

        }

    }
    
    // Commands ////////////////////////////////////////////////////////////////////////////////////

    public void teleop() throws InterruptedException {

        /*
        *
        *   WHEEL POWER
        *
        *   This is how we apply power to our wheels on the first controller.
        *   By the values reading from the left and right stick's y-values (as this is tank drive)
        *   it scales the amount of power to apply to the motors using our scaleInput() method, and
        *   applies it to the motors. It works in all directions/values.
         */

        bL.setPower(scaleInput(gamepad1.left_stick_y));
        fL.setPower(scaleInput(gamepad1.left_stick_y));

        bR.setPower(scaleInput(gamepad1.right_stick_y));
        fR.setPower(scaleInput(gamepad1.right_stick_y));

        // TEMP GYRO ------------------------------------------------------------------------------

        /*

            This returns the different values of the IMU unit to show how far we have moved. It's
            printing back to the Driver Station.

         */

        /*
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
        telemetry.addData("Pitches: ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));
                */


        // Cow Catchers ////////////////////////////////////////////////////////////////////////////


        if(gamepad1.left_trigger > 0 ){

            cowLeft.setPosition(gamepad1.left_trigger);

        } else {

            cowLeft.setPosition(0);

        }

        if(gamepad1.right_trigger > 0) {

            cowRight.setPosition(1 - gamepad1.right_trigger);

        }else{

            cowRight.setPosition(1);

        }


        if(gamepad1.y) {

            cowLeft.setPosition(cowLeftOpen);
            cowRight.setPosition(cowRightOpen);


        }else if(gamepad1.a){

            cowLeft.setPosition(0);
            cowRight.setPosition(1);

        }


        // Dispenser Arms //////////////////////////////////////////////////////////////////////////

        /*
            The dispenser arms are controlled by two motors that are run with encoders.

            On the second gamepad, the dpad values for up and down control whether the
            arms move (in sync) forward or backwards until it reaches a set encoder count.
            dispPosition is the lowest/farthest out we want the dispenser to go (which by a
            matter of fact is the high goal on the ramp) and the other position is 0, which is
            naturally selected as the resting position of the motors when the program starts
            (wherever the motors are when the program is started is considered encoder position
            zero).

            This allows us to use the motors as standard (not continuous) servos with much more
            power and the ability to use any angle range we want. We measure the 'angle' in encoder
            counts.

            dispPower is a constant in our RobotOpMode class that is used to specify how fast we
            want the dispenser arms to travel. Having it there makes it easier to change the value
            in all of our TeleOp and Autonomous programs on a global scale.
         */


        if(gamepad2.dpad_up){

            dispL.setTargetPosition(dispPosition);
            dispR.setTargetPosition(dispPosition);

            dispL.setPower(dispPower);
            dispR.setPower(dispPower);

        }else if(gamepad2.dpad_down){

            dispL.setTargetPosition(0);
            dispR.setTargetPosition(0);

            dispL.setPower(-dispPower);
            dispR.setPower(-dispPower);

        }else{

            dispL.setPower(0);
            dispR.setPower(0);

        }

        /*
        *   TILTING CONTROL
        *
        *   This is a variable control for the tilting of the debris bucket.
        *   This allows us to tilt the bucket as little or as much as we want, using the left
        *   joystick on the second controller. Moving it left and right (x position) moves it
        *   further in the corresponding direction.
        *
         */

        if(gamepad2.left_stick_x < -0.1 || gamepad2.left_stick_x > 0.1) {
            tilt.setPosition((gamepad2.left_stick_x + 1) / 2);
        }else{
            tilt.setPosition(tiltMiddle);
        }


        /*
        *
        *   RELEASING THE DEBRIS
        *
        *   We have two servos (connected with a y-connector to one port) that are mounted reverse
        *   to each other so they can be controlled with the same values.
        *   Setting to one position or the other depending on a button press, we can open or close
        *   the flaps on the sides of the bucket to release or contain the debris.
        *   Pushing the button releases them. If it's unpressed it contains the blocks.
        *
        *
         */

        if(gamepad2.a){
            release.setPosition(releaseOpen);
        }else{
            release.setPosition(releaseClosed);
        }


        /*
        *
        *   CLIMBER RELEASE
        *
        *   This is very similar to the debris release above.
        *   This case instead is with the servo to control the retention of the climbers in our
        *   custom storage unit.
        *
        *
         */
        if(gamepad2.y){
            climbers.setPosition(climbersOpen);
        }else{
            climbers.setPosition(climbersClosed);
        }


        /*
        *
        *   HARVESTER CONTROL
        *
        *   This supplies variable power to the motor controlling the debris collector with
        *   our zip ties.
        *   Similar to wheel power.
         */
        harvester.setPower(scaleInput(gamepad2.right_stick_y));


    }// end of teleop()



}

