package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 11/30/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Framework", group="Autonomous")
//@Disabled
public class AutoFramework extends LinearOpMode{
    //Initialize all of your motors, servos, sensors
    /*
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor shooter = null;
    Servo particleLift = null;
    Servo ccLeft = null;
    Servo ccRight = null;
    private ElapsedTime runtime = new ElapsedTime();
    */

    /* Declare OpMode members. */
    //HardwarePushbot robot   = new HardwarePushbot();   // When you guys make a hardware class, this is for constructing
    //private ElapsedTime     runtime = new ElapsedTime();

    //Place constants here e.g. encoder counts!

    //Here's where the magic actually happens lol
    public void runOpMode() {
        //Here's an example of telemetry, this displays on the phone
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //make calls to the hardware map (this is just a formality, dw bout what it actually do)
      /*leftMotor = hardwareMap.dcMotor.get("left motor");
        particleLift = hardwareMap.servo.get("particle lift");*/

        //ensure everything is going in the direction you want
     /* leftMotor.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

        //when using encoders, do this, don't ask why (check the yellow postit on the desktop)
      /*leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        //set the initial values for ya servos
        //servos work for values from 0-1, you can use the MR program to determine
        //values from 0-255 and use the corresponding fraction:
      /*particleLift.setPosition(250. / 255.);*/

        //set the power for all of your motors to 0. why? because i said so. don't make robo move just yet
       /*leftMotor.setPower(0.);*/

        //Here's the ticket yo, everything before this is initialization, and after this is all of
        //the actual robot-moving stuff
        waitForStart();
        //THIS IS THE IMPORTANT PART AND I'M TYPING IN CAPS SO THAT YOU NOTICE THIS HOPEFULLY k.
        //In here, you put the actual code. My recommendation is that you write methods to do the bulk
        //of the work, and then insert in between steps
        //e.g. drive off base, then call findTape(), etc
    }

    //METHODS
    public void moveThatRobot(double speed, double leftInches, double rightInches, double timeout){
        //example shell of a method for y'all to use :) you're welcome
    }

}


