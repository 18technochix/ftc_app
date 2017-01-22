//teleop template

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@TeleOp(name="MecanumDrive", group="Linear Opmode")
@Disabled
public class MecanumDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor backLeft = null;
    DcMotor backRight = null;
    DcMotor frontLeft = null;
    DcMotor frontRight = null;

    /*

        Create any sensors / motors here.

        ex. TouchSensor frontTouch;

        The first section is the type of object you are creating (ex. Motor, LightSensor, TouchSensor.
        The second section is whatever you want to name that object.

     */

    public void runOpMode() throws InterruptedException{ //Initializes/runs the program.

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        backLeft = hardwareMap.dcMotor.get("bL");
        backRight = hardwareMap.dcMotor.get("bR");
        frontLeft = hardwareMap.dcMotor.get("fL");
        frontRight = hardwareMap.dcMotor.get("fR");

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        runtime.reset();

        while(opModeIsActive()) { //While the program is running, do this.

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

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




            frontLeft.setPower(0.5*scale(ly, rx, lx));
            backLeft.setPower(0.5*scale(ly, rx, -lx));
            frontRight.setPower(0.5*scale(ly, -rx, -lx));
            backRight.setPower(0.5*scale(ly, -rx, lx));

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
            waitForTick(40);
            idle();
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
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)runtime.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        runtime.reset();
    }
}

