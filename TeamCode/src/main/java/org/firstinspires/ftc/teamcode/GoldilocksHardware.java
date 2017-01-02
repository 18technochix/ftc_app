package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

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
    public ColorSensor whiteLineSensorOne = null;
    public ColorSensor whiteLineSensorTwo = null;
    public ColorSensor colorBlue = null;
    public ColorSensor colorRed = null;

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
    public static double wallTouchMinus;

    public static boolean redTrue;                              //boolean for color value
    public static boolean blueTrue;

    public static final int maxBop = 2500;


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
        touchBlue = hwMap.touchSensor.get("touch left");
        //touchRed = hwMap.touchSensor.get("touch right");
        //cdim = hwMap.lightSensor.get("color sensor left");
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

        //SENSORS STILL UNDER TESTING
        //cdim = hwMap.deviceInterfaceModule.get("dim");
    }

    //AUTONOMOUS INIT
    public void autoInit(HardwareMap someHwMap){
        init(someHwMap);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        buttonBopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //encoder setup
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        buttonBopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        buttonBopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

}
