package org.firstinspires.ftc.teamcode;

//import statements
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Techno Team_PC_III on 12/2/2017.
 */

@Disabled
public class Hardware{
    //public OpMode members
    DcMotor fr = null;
    DcMotor fl = null;
    DcMotor bl = null;
    DcMotor br = null;
    DcMotor lift = null;
    DcMotor bopperMotor = null;
    Servo grabServo = null;
    Servo jewelServo = null;

    BNO055IMU gyro;
    ColorSensor jewelSensor = null;
    ColorSensor tapeSensor1 = null;
    ColorSensor tapeSensor2 = null;
    VuforiaLocalizer picReader = null;

    //constants
    double GRAB_OPEN = 0.12;
    double GRAB_CLOSE = 0.63;
    double servoPosition = ((GRAB_CLOSE - GRAB_OPEN)/2) + GRAB_OPEN;
    int redMin = 355;
    int redMax = 10;
    int blueMin = 221;
    int blueMax = 240;
    int bopperEncoderCount = 50;
    //local OpMode members
    HardwareMap hwMap           = null;
    LinearOpMode opMode         = null;
    private Telemetry telemetry = null;

    //constructor
    public Hardware(LinearOpMode _opMode){
        opMode = _opMode;
        telemetry = _opMode.telemetry;
    }

    //init methods
    //BASE INIT
    public void init(HardwareMap someHwMap){
        hwMap = someHwMap;
        fr = hwMap.dcMotor.get("fr");
        br = hwMap.dcMotor.get("br");
        fl = hwMap.dcMotor.get("fl");
        bl = hwMap.dcMotor.get("bl");
        lift = hwMap.get(DcMotor.class, "lift");
        bopperMotor = hwMap.dcMotor.get("bopperMotor");
        grabServo = hwMap.get(Servo.class, "grabServo");
        jewelServo = hwMap.get(Servo.class, "jewelServo");
        gyro = hwMap.get(BNO055IMU.class, "imu");

        jewelSensor = hwMap.colorSensor.get("jewelSensor");
        tapeSensor1 = hwMap.colorSensor.get("tapeSensor1");
        tapeSensor2 = hwMap.colorSensor.get("tapeSensor2");
        hwMap.get(BNO055IMU.class, "imu");
        picReader = hwMap.get(VuforiaLocalizer.class, "picReader");

        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);

        grabServo.setDirection(Servo.Direction.FORWARD);
        grabServo.setPosition(servoPosition);

        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fr.setPower(0.);
        br.setPower(0.);
        fl.setPower(0.);
        bl.setPower(0.);

        gyro = null; //??? do we need this
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = "ARYJT0b/////AAAAGYhN7cav+UUXqkMo7uS9Mswt0KxiQ3Sp/OVgoLfwHMP74uJpsnWLAXQLoXs0AIcpgC2IiJIov+JwDwrMwujShtlUastkjxWBAXLvJ6drxd811wEZGqBtBeOC6ObFPqG+W41u3D0fWJjsU4qG3S6NdgIAv6Q4T1OGH6Q6jOpatGlpEyhclM0Rk+vs77zaVzgBgZmcCa+tTqOpu0hhxqyxMvPv3Ehn0sgbF1KTfba/QQfxEjpsqJRyA5r7HfNNfg/31xdLLtzQXy28id0EXqPkB2iZ39fxsX0XcbKRWd7pq5uXqfvwJm4EvsKFLOz0eJhJBW+2vlCy5jrdehA7wH+pOnQTx3SQmbyqlr8KehWPWL1X";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.picReader = ClassFactory.createVuforiaLocalizer(params);
        VuforiaTrackables relicTrackables = this.picReader.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();


        Orientation angles;

    }

    public void autoInit(HardwareMap someHwMap){
        init(someHwMap);
    }

    public void teleInit(HardwareMap someHwMap){
        init(someHwMap);
    }

    public double getHeading(){
        Orientation angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double angle = angles.firstAngle;
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle));
    }





}
