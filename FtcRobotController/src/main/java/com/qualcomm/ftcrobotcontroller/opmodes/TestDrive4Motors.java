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

    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    DcMotor armLeft;
    DcMotor armRight;

    Servo left;
    Servo right;

    Servo beacon;

    //Variables
    final double leftDown = 0.5; // value for left servo's extended position
    final double leftUp = 0.25; //value for left servo's retracted position

    final double rightDown = 0; //value for right servo's extended position
    final double rightUp = 0.25; //value for right servo's retracted position

    /*

        Create any sensors / motors here.

        ex. TouchSensor frontTouch;

        The first section is the type of object you are creating (ex. Motor, LightSensor, TouchSensor.
        The second section is whatever you want to name that object.

     */


    public void runOpMode() throws InterruptedException{ //Initializes/runs the program.


        backLeft = hardwareMap.dcMotor.get("bL");
        backRight = hardwareMap.dcMotor.get("bR");
        frontLeft = hardwareMap.dcMotor.get("fL");
        frontRight = hardwareMap.dcMotor.get("fR");

        armLeft = hardwareMap.dcMotor.get("aL");
        armRight = hardwareMap.dcMotor.get("aR");

        left = hardwareMap.servo.get("sL");
        right = hardwareMap.servo.get("sR");

        beacon = hardwareMap.servo.get("beacon");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        armRight.setDirection(DcMotor.Direction.REVERSE);

        armLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

//        resetEncoders();

        left.setPosition(leftDown);
        right.setPosition(rightDown);

        /*

            Initialize the sensors / motors here.

            ex. frontTouch = hardwareMap.TouchSensor.get("name of sensor in the configuration");

            the name of the sensor in the quotes will be whatever you call the sensor in
            the configuration of the motors and sensors on the robot controller. it must be exactly
            what you name it.

         */

        waitForStart(); //This waits for the driver station to say "start the program!".

        while(opModeIsActive()) { //While the program is running, do this.

            // Controls for the wheels

            backLeft.setPower(scaleInput(gamepad1.left_stick_y));
            frontLeft.setPower(scaleInput(gamepad1.left_stick_y));

            backRight.setPower(scaleInput(gamepad1.right_stick_y));
            frontRight.setPower(scaleInput(gamepad1.right_stick_y));

            //Controls for the servos
            // The dpad controls the left side, the buttons control the right

            if(gamepad2.dpad_left){ //if left dpad is pressed, move left servo to the outside position

                left.setPosition(leftUp);

            }else if(gamepad2.dpad_right){ //if the right dpad is pressed, move left servo to inward position

                left.setPosition(leftDown);

            }

            if(gamepad2.b){

                right.setPosition(rightUp);

            }else if(gamepad2.x){

                right.setPosition(rightDown);

            }

            if(gamepad2.dpad_down){
                armLeft.setTargetPosition(0);
                armLeft.setPower(0.15);
            }else if(gamepad2.dpad_up){
                armLeft.setTargetPosition(500);
                armLeft.setPower(0.15);
            }
            else{
              armLeft.setPower(0.);
              }

            if(gamepad2.a){
                armRight.setTargetPosition(0);
                armRight.setPower(0.15);
            }else if(gamepad2.y){
                armRight.setTargetPosition(500);
                armRight.setPower(0.15);
            }
            else{
                armRight.setPower(0.);
            }


            /*

                Write any code here that allows the joysticks/buttons to control the motors and
                sensors (if/while statements).

                If you want to send information back to the driver station, you must submit it
                through the telemetry.

                ex. telemetry.addData("Front Touch Pressed:" , String.valueOf(frontTouch.isPressed()));

                The value in the quotes is what the value will be named/displayed as on the phone.
                After the comma is the number/value (which must be converted to a string) that
                will change depending on the sensor.

             */
            waitOneFullHardwareCycle();
        }

    }

    public void resetEncoders() {
        armLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}

