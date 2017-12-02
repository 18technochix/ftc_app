package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 11/30/2017.
 */

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Auto Framework", group="Autonomous")
//@Disabled
public class AutoFramework extends LinearOpMode{
    //Initialize all of your motors, servos, sensors

    DcMotor fr = null;
    DcMotor fl = null;
    DcMotor bl = null;
    DcMotor br = null;

    DcMotor shooter = null;
    Servo particleLift = null;
    Servo ccLeft = null;
    Servo ccRight = null;
    ColorSensor colorSensor = null;
    BNO055IMU gyro;


    private ElapsedTime runtime = new ElapsedTime();




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
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("fbl");



        /*particleLift = hardwareMap.servo.get("particle lift");*/
        colorSensor = hardwareMap.colorSensor.get("TapeSensor");
        hardwareMap.get(BNO055IMU.class, "imu");

        gyro = null;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);

        Orientation angles;



        //ensure everything is going in the direction you want
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        /*shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

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
       fr.setPower(0.);
       br.setPower(0.);
       fl.setPower(0.);
       bl.setPower(0.);

        //Here's the ticket yo, everything before this is initialization, and after this is all of
        //the actual robot-moving stuff
        waitForStart();
        telemetry.addData("Heading", getHeading());
        //THIS IS THE IMPORTANT PART AND I'M TYPING IN CAPS SO THAT YOU NOTICE THIS HOPEFULLY k.
        //In here, you put the actual code. My recommendation is that you write methods to do the bulk
        //of the work, and then insert in between steps
        //e.g. drive off base, then call findTape(), etc
        spinRobot(180, 0.5);
    }

    //METHODS
    public void moveThatRobot(double speed, double leftInches, double rightInches, double timeout){
        //example shell of a method for y'all to use :) you're welcome
    }
    //returns the color of the Jewels or the balancing stone

    public int getColor(ColorSensor sensor){
        float hsvValues[] = {0F,0F,0F};
        float values[] = hsvValues;
        boolean LedOn = true;
        sensor.enableLed(LedOn);
        Color.RGBToHSV((sensor.red() * 255) / 800, (sensor.green() * 255) / 800, (sensor.blue() * 255) / 800, hsvValues);
        telemetry.addData("LED", LedOn ? "On" : "Off");
        telemetry.addData("Clear", sensor.alpha());
        telemetry.addData("Red  ", sensor.red());
        telemetry.addData("Green", sensor.green());
        telemetry.addData("Blue ", sensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
       int color =  Color.HSVToColor(0xff, values);

        return color;


    }
    public double getHeading(){
        Orientation angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double angle = angles.firstAngle;
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle));
    }

        public void spinRobot(double angle, double power){
            while(getHeading()< angle ) {
                fr.setPower(power);
                br.setPower(power);
                fl.setPower(-power);
                fr.setPower(-power);

            }
            power = 0;
            fr.setPower(power);
            br.setPower(power);
            fl.setPower(-power);
            fr.setPower(-power);

        } //timeout


}

