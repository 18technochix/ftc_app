package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Techno Team_PC_III on 12/23/2016.
 */

public class GoldilocksHardware {

    //public OpMode members
    public DcMotor     leftMotor       = null;
    public DcMotor     rightMotor      = null;
    public DcMotor     shooter         = null;
    public DcMotor     collector       = null;
    public DcMotor     buttonBopper    = null;

    public Servo       particleLift    = null;
    public Servo       ccLeft          = null;
    public Servo       ccRight         = null;

    public TouchSensor touchBlue = null;
    public TouchSensor touchRed = null;
    public LightSensor whiteLineSensorOne = null;
    //public LightSensor whiteLineSensorTwo = null;
    public ColorSensor colorBlue = null;
    public ColorSensor colorRed = null;
    public BNO055IMU gyro = null;

    //SENSORS STILL UNDER TEST
    DeviceInterfaceModule cdim;

    //declare variables & give values if necessary
    public static final double ccLeftClose = (2./255.);        //cowcatcher open/close values
    public static final double ccRightClose = (220./255.);
    public static final double ccLeftOpen = (200./255.);
    public static final double ccRightOpen = (50./255.);

    public static final double particleLiftUp = (175./255.);    //up/down particleLift positions
    public static final double particleLiftDown = (250./255.);

    public static boolean leftOpen;                             //individual cowcatcher control
    public static boolean rightOpen;

    public static int wallTouch;                            //have we touched the wall
    public static final int beaconDepth = 750;
    public static final int beaconClearance = 1750;
    public static double driveCorrect;

    public static final int maxBop = 3300;

    public static final double redHue = 346.;
    public static final double blueHue = 234.;
    public static final double midHue = (blueHue + redHue)/2;
    public static final double lineLight = .350;


    //auto constants
    static final double     ENCODER_CPR             = 1120 ;    // AndyMark encoder count
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (ENCODER_CPR * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     SHOOTER_SPEED           = 0.2;
    static final double     TURN_SPEED              = 0.2;

    static final double CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER_INCHES;

    // local OpMode members
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // Constructor
    public GoldilocksHardware(){

    }

    //BASE INIT METHOD
    public void init(HardwareMap someHwMap){
        hwMap = someHwMap;

        //name the motors
        leftMotor  = hwMap.dcMotor.get("left motor");
        rightMotor = hwMap.dcMotor.get("right motor");
        shooter = hwMap.dcMotor.get("shooter");
        buttonBopper = hwMap.dcMotor.get("button bopper");
        collector = hwMap.dcMotor.get("collector");
        touchRed = hwMap.touchSensor.get("touch left");
        touchBlue = hwMap.touchSensor.get("touch right");
        colorBlue = hwMap.colorSensor.get("color sensor left");
        whiteLineSensorOne = hwMap.lightSensor.get("line sensor");
        gyro = hwMap.get(BNO055IMU.class, "imu");


        //set direction
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //set power
        leftMotor.setPower(0.);
        rightMotor.setPower(0.);
        shooter.setPower(0);
        buttonBopper.setPower(0);
        collector.setPower(0);

        //name the servos
        ccLeft = hwMap.servo.get("cc left");
        ccRight = hwMap.servo.get("cc right");
        particleLift = hwMap.servo.get("particle lift");
        //set position
        particleLift.setPosition(particleLiftDown);
        ccRight.setPosition(ccRightClose);
        ccLeft.setPosition(ccLeftClose);

       whiteLineSensorOne.enableLed(true);
    }

    //AUTONOMOUS INIT
    public void autoInit(HardwareMap someHwMap){
        init(someHwMap);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //encoder setup
        //leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        buttonBopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);

    }

    //TELEOP INIT
    public void teleInit(HardwareMap someHwMap) {
        init(someHwMap);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //run without encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //buttonBopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //individual cowcatcher control booleans
        leftOpen = false;
        rightOpen = false;

    }

    public float getBlueHue(){
        float hsvValues[] = {0F,0F,0F};
        Color.RGBToHSV((colorBlue.red() * 255) / 800, (colorBlue.green() * 255) / 800, (colorBlue.blue() * 255) / 800, hsvValues);
        return hsvValues [0];
    }

    public float getHeading(){
        Orientation angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        float angle = angles.firstAngle;
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle));
        //return angles.firstAngle;
    }

    public int inchToEncoder(double inches){
     return (int)(inches * GoldilocksHardware.COUNTS_PER_INCH);
    }
}
