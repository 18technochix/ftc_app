//teleop template

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Techno Team_PC_III on 9/27/2015.
 */

/*
fl = ly + rx + lx
rl = ly + rx - lx
fr = ly - rx - lx
rr = ly - rx + lx
 */
// after class, replace "TeleOpTemplate" with your class/"program" name.
public class MecanumDrive extends LinearOpMode{

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


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        /*

            Initialize the sensors / motors here.

            ex. frontTouch = hardwareMap.TouchSensor.get("name of sensor in the configuration");

            the name of the sensor in the quotes will be whatever you call the sensor in
            the configuration of the motors and sensors on the robot controller. it must be exactly
            what you name it.

         */

        waitForStart(); //This waits for the driver station to say "start the program!".

        while(opModeIsActive()) { //While the program is running, do this.

            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;

            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;

            /*
            fl = ly + rx + lx
            rl = ly + rx - lx
            fr = ly - rx - lx
            rr = ly - rx + lx
            */




            frontLeft.setPower(scale(ly, rx, lx));
            backLeft.setPower(scale(ly, rx, (-1 * lx)));
            frontRight.setPower(scale(ly, (-1* rx), (-1* lx)));
            backRight.setPower(scale(ly, (-1 * rx), lx));

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

    public double scale(double one, double two, double three){

        if(one + two + three > 1){

            return 1;

        }else if(one + two + three < -1){

            return -1;

        }else{

            return one + two + three;

        }

    }

}

