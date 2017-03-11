package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * Created by Techno Team_PC_III on 12/23/2016.
 */

public class GoldilocksHardware {

    //public OpMode members
    public DcMotor     leftMotor       = null;
    public DcMotor     rightMotor      = null;
    public DcMotor     shooter         = null;
    public DcMotor     collector       = null;
    public DcMotor     buttonBopper    = null;
    public DcMotor     gathererSpinner = null;
    public DcMotor     gathererArm     = null;

    public Servo       particleLift    = null;
    public Servo       ccLeft          = null;
    public Servo       ccRight         = null;

    public LightSensor whiteLineSensorOne = null;
    //public LightSensor whiteLineSensorTwo = null;
    public ColorSensor colorBlue = null;
    public ColorSensor colorRed = null;
    public BNO055IMU gyro = null;
    public UltrasonicSensor lds = null;

    //declare variables & give values if necessary
    public static final double ccLeftClose = (2./255.);        //cowcatcher open/close values
    public static final double ccRightClose = (220./255.);
    public static final double ccLeftOpen = (200./255.);
    public static final double ccRightOpen = (50./255.);

    public static final double particleLiftUp = (175./255.);    //up/down particleLift positions
    public static final double particleLiftDown = (250./255.);

    public static boolean leftOpen;                             //individual cowcatcher control
    public static boolean rightOpen;

    public static int wallTouch;
    public static final int encoderPerInch = 940;      // encoder counts per inch
    public static final int beaconDepth = (int)(1.5*(double)encoderPerInch); //750, 725, 700, 650
    public static final double wallGap = 8;
    public static double driveCorrect;

    public static final int maxBop = 3680;
    public static final int bopperSensorSpace = (int)(7.5*(double)encoderPerInch);
    public static final int bopperOvershoot = (int)(0.15*(double)encoderPerInch);
    static final int bopperRetract = (int)(.75*(double)encoderPerInch);
    static final int bopperWidth = (int)(17.25*(double)encoderPerInch);

    public static final double redHue = 346.;
    public static final double blueHue = 234.;
    public static final double midHue = (blueHue + redHue)/2.;
    public static final double lineLight = .3;
    public static final double shooterIncrement = .01;
    public double shooterPower = 0;


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
    public /*private*/ ElapsedTime runtime  = new ElapsedTime();
    LinearOpMode opMode = null;
    private Telemetry telemetry = null;

    // Constructor
    public GoldilocksHardware(LinearOpMode _opMode){
        opMode = _opMode;
        telemetry = _opMode.telemetry;
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
        gathererSpinner = hwMap.dcMotor.get("gatherer spinner");
        gathererArm = hwMap.dcMotor.get("gatherer arm");
        colorBlue = hwMap.colorSensor.get("color sensor left");
        whiteLineSensorOne = hwMap.lightSensor.get("line sensor");
        gyro = hwMap.get(BNO055IMU.class, "imu");


        //set direction
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        buttonBopper.setDirection(DcMotorSimple.Direction.REVERSE);
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
    void autoInit(HardwareMap someHwMap, boolean isBlue){
        init(someHwMap);

        if (isBlue) {
            lds = hwMap.ultrasonicSensor.get("lds right");
        } else {
            lds = hwMap.ultrasonicSensor.get("lds left");
        }

        //reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        buttonBopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        //run without encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //individual cowcatcher control booleans
        leftOpen = false;
        rightOpen = false;

    }

    public float getBlueHue(){
        float hsvValues[] = {0F,0F,0F};
        Color.RGBToHSV((colorBlue.red() * 255) / 800, (colorBlue.green() * 255) / 800, (colorBlue.blue() * 255) / 800, hsvValues);
        return hsvValues [0];
    }

    public double getHeading(){
        Orientation angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double angle = angles.firstAngle;
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle));
    }

    public Position getPosition(){
        return gyro.getPosition();
    }

    public double getDistance(){
        return lds.getUltrasonicLevel();
    }

    public int inchToEncoder(double inches){
     return (int)(inches * GoldilocksHardware.COUNTS_PER_INCH);
    }

    public void setLeftPower(double p){
        leftMotor.setPower(p);
    }

    public void setRightPower(double p){
        rightMotor.setPower(p * .95); //.92
    }

    public void stopDriveMotors(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void rampUpShooter(double t) {
        shooterPower = 0.;
        while (shooterPower < t) {
            shooterPower = shooterPower + shooterIncrement;
            shooter.setPower(shooterPower);
            sleep(20);
        }

        sleep(2000); //sleep for a moment just to make sure the shooter is up to speed (1500)abd
    }

    public void doubleShot() {

        particleLift.setPosition(particleLiftUp); //190
        sleep(500);     // pause for servos to move
        particleLift.setPosition(particleLiftDown);
        sleep(1500);     // pause for servos to move
        particleLift.setPosition(particleLiftUp);
        sleep(500);     // pause for servos to move
    }

    public void rampDownShooter(){
        while (shooterPower > 0.) {
            shooterPower = shooterPower - shooterIncrement;
            shooter.setPower(Math.abs(shooterPower));
            sleep(20);
        }
        shooter.setPower(0.);
    }

    public void runShooter(double t){
        rampUpShooter(t);
        doubleShot();
        rampDownShooter();
    }
    private void sleep(long ms){
        opMode.sleep(ms);
    }

    private boolean opModeIsActive(){return opMode.opModeIsActive();}

    private void idle(){opMode.idle();}

    public void moveThatRobot(double speed, double leftInches, double rightInches, double timeout, String tag){
        moveThatRobot(speed, speed, leftInches, rightInches, timeout, tag);
    }

    public void moveThatRobot(double leftSpeed, double rightSpeed, double leftInches, double rightInches, double timeout, String tag){
        int newLeftTarget;
        int newRightTarget;
        int lCurrent;
        int rCurrent;
        double lComplete;
        double rComplete;
        double lSlowdown;
        double rSlowdown;

        //are we still running? good. if so:
        if (opModeIsActive()) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //now, where do we go? let's set the target position.
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * GoldilocksHardware.COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * GoldilocksHardware.COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            //now you gotta make sure they know what to do with this info. give the motor a runmode.
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(leftSpeed));
            rightMotor.setPower(Math.abs(rightSpeed));
            double lastSpeedSetTime = runtime.seconds();

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftMotor.isBusy() || rightMotor.isBusy())) {
                //((AutoBeaconBase)opMode).ultrasonicDriveCorrect(wallGap, leftSpeed, rightSpeed, .9);

                lCurrent = leftMotor.getCurrentPosition();
                rCurrent = rightMotor.getCurrentPosition();

                // Display it for the driver.
                telemetry.addData(tag + ": Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData(tag + ": Path2", "Running at %7d :%7d", lCurrent, rCurrent);
                telemetry.update();

                lComplete = (double) lCurrent / (double) newLeftTarget;
                rComplete = (double) rCurrent / (double) newRightTarget;

                lSlowdown = 1.0;
                if (lComplete > 0.92) {
                    lSlowdown = 0.5;
                } else if (lComplete > 0.85) {
                    lSlowdown = 0.8;
                }
                rSlowdown = 1.0;
                if (rComplete > 0.92) {
                    rSlowdown = 0.5;
                } else if (rComplete > 0.85) {
                    rSlowdown = 0.8;
                }

                if ((runtime.seconds() - lastSpeedSetTime) > 0.04) {
                    leftMotor.setPower(lSlowdown * Math.abs(leftSpeed));
                    rightMotor.setPower(rSlowdown * Math.abs(rightSpeed));
                    lastSpeedSetTime = runtime.seconds();
                }

                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0.);
            rightMotor.setPower(0.);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


}
