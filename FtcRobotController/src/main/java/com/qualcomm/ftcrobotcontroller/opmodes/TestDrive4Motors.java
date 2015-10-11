//teleop template

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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


        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        /*

            Initialize the sensors / motors here.

            ex. frontTouch = hardwareMap.TouchSensor.get("name of sensor in the configuration");

            the name of the sensor in the quotes will be whatever you call the sensor in
            the configuration of the motors and sensors on the robot controller. it must be exactly
            what you name it.

         */

        waitForStart(); //This waits for the driver station to say "start the program!".

        while(opModeIsActive()) { //While the program is running, do this.


            if(gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1){

                backLeft.setPower(1 * gamepad1.left_stick_y);
                frontLeft.setPower(1 * gamepad1.left_stick_y);

            }else{

                backLeft.setPower(0);
                frontLeft.setPower(0);
                

            }

            if(gamepad1.right_stick_y > .1 || gamepad1.right_stick_y < -.1){

                backRight.setPower(1 * gamepad1.right_stick_y);
                frontRight.setPower(1 * gamepad1.right_stick_y);

            }else{

                backRight.setPower(0);
                frontRight.setPower(0);

            }

            /*

                Write any code here that allows the joysticks/buttons to control the motors and
                sensors (if/while statments).

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

}

