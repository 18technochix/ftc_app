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
public class ServoTest extends LinearOpMode{

    Servo one;
    Servo two;

    DcMotor three;
    DcMotor four;
    DcMotor five;

    /*

        Create any sensors / motors here.

        ex. TouchSensor frontTouch;

        The first section is the type of object you are creating (ex. Motor, LightSensor, TouchSensor.
        The second section is whatever you want to name that object.

     */

    public void runOpMode() throws InterruptedException{ //Initializes/runs the program.


        one = hardwareMap.servo.get("one");
        two = hardwareMap.servo.get("two");

        three = hardwareMap.dcMotor.get("three");
        four = hardwareMap.dcMotor.get("four");
        five = hardwareMap.dcMotor.get("five");
        /*

            Initialize the sensors / motors here.

            ex. frontTouch = hardwareMap.TouchSensor.get("name of sensor in the configuration");

            the name of the sensor in the quotes will be whatever you call the sensor in
            the configuration of the motors and sensors on the robot controller. it must be exactly
            what you name it.

         */

        waitForStart(); //This waits for the driver station to say "start the program!".

        while(opModeIsActive()){ //While the program is running, do this.


            if(gamepad1.dpad_right){

                one.setPosition(.3);
                two.setPosition(.3);

                three.setPower(.7);
                four.setPower(.7);
                five.setPower(.7);

            } else {


                one.setPosition(1);
                two.setPosition(1);

                three.setPower(0);
                four.setPower(0);
                five.setPower(0);

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

