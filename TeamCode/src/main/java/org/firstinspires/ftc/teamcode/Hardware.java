package org.firstinspires.ftc.teamcode;

//import statements
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    Servo grabServo = null;

    BNO055IMU gyro;
    ColorSensor colorSensor = null;

    //constants
    double GRAB_OPEN = 0.12;
    double GRAB_CLOSE = 0.63;
    double servoPosition = ((GRAB_CLOSE - GRAB_OPEN)/2) + GRAB_OPEN;

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
        grabServo = hwMap.get(Servo.class, "grabServo");
        gyro = hwMap.get(BNO055IMU.class, "imu");

        colorSensor = hwMap.colorSensor.get("TapeSensor");
        hwMap.get(BNO055IMU.class, "imu");

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
