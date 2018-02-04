package org.firstinspires.ftc.teamcode;

//import statements

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Techno Team_PC_III on 12/2/2017.
 */

@Disabled
public class Hardware{
    //public OpMode members
    public enum Servos {
        RELIC_EXTEND_ARM,
        //RELIC_EXTEND_ARM2,
        RELIC_ELBOW,
        RELIC_WRIST,
        RELIC_GRAB
    }
    //motors
    DcMotor fr = null;
    DcMotor fl = null;
    DcMotor bl = null;
    DcMotor br = null;
    DcMotor lift = null;
    //servos
    Servo glyphGrab = null;
    Servo jewelServoBlue = null;
    Servo jewelServoRed= null;
    CRServo relicExtendArm1 = null;
    CRServo relicExtendArm2 = null;
    CRServo relicElbow = null;
    Servo relicWrist = null;
    Servo relicGrab = null;
    Servo glyphPush = null;
    //sensors
    BNO055IMU imu = null;
    BNO055IMU.Parameters parameters = null;
    MultiplexColorSensor colorSensor = null;
    VuforiaLocalizer picReader = null;
    VuforiaTrackable relicTemplate = null;
    //constants
    double GLYPH_GRAB_OPEN = 1.0;
    double GLYPH_GRAB_CLOSE = 0.0;
    double glyphGrabPosition = GLYPH_GRAB_OPEN;

    double GLYPH_PUSH_OUT = 1.0;
    double GLYPH_PUSH_IN = 0.0;
    double glyphPushPosition = GLYPH_PUSH_IN;

    double RELIC_WRIST_UP = 1.0;
    double RELIC_WRIST_MID = 0.5;
    double RELIC_WRIST_DOWN = 0.0;
    double relicWristPosition = RELIC_WRIST_MID;

    double RELIC_GRAB_OPEN = 1.0;
    double RELIC_GRAB_CLOSE = 0.0;
    double relicGrabPosition = RELIC_GRAB_CLOSE;

    double RELIC_ELBOW_STOP = 0.0 - 2.0/255.;

    double JEWEL_DOWN = 0.0;
    double JEWEL_UP = 1.0;
    double JEWEL_UP_TAD = 0.3;
    double redMin = 0;
    double redMax = 20;
    double blueMin = 125 ;
    double blueMax = 360;
    int bluePort = 2;
    int redPort = 1;
    int tapeSensorLeft = 3;
    int tapeSensorRight = 0;
    static final int colorSampleMilliseconds = 48;
    final double rightAdjustment = 12.5/70.0;

    static final double     ENCODER_CPR             = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (ENCODER_CPR * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //local OpMode members
    HardwareMap hwMap           = null;
    LinearOpMode opMode         = null;
    private Telemetry telemetry = null;
    public ElapsedTime runtime  = new ElapsedTime();

    //constructor
    public Hardware(LinearOpMode _opMode){
        opMode = _opMode;
        telemetry = _opMode.telemetry;
    }

    //init methods
    //BASE INIT
    public void init(HardwareMap someHwMap){
        hwMap = someHwMap;
        //motors
        fr = hwMap.dcMotor.get("fr");
        br = hwMap.dcMotor.get("br");
        fl = hwMap.dcMotor.get("fl");
        bl = hwMap.dcMotor.get("bl");
        lift = hwMap.get(DcMotor.class, "lift");
        //servos
        glyphGrab = hwMap.get(Servo.class, "glyphGrab");
        glyphPush = hwMap.get(Servo.class, "glyphPush");
        jewelServoBlue = hwMap.get(Servo.class, "jewelServoBlue");
        jewelServoRed = hwMap.get(Servo.class, "jewelServoRed");
        relicExtendArm1 =hwMap.get(CRServo.class, "relicExtendArm1");
        relicExtendArm2 =hwMap.get(CRServo.class, "relicExtendArm2");
        relicElbow =hwMap.get(CRServo.class, "relicElbow");
        relicWrist = hwMap.get(Servo.class, "relicWrist");
        relicGrab = hwMap.get(Servo.class, "relicGrab");

        //sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        int[] ports = {bluePort, redPort, tapeSensorLeft, tapeSensorRight};
        colorSensor = new MultiplexColorSensor(someHwMap, "mux", "color sensor", ports, colorSampleMilliseconds,
                MultiplexColorSensor.GAIN_16X);

        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);

        glyphGrab.setDirection(Servo.Direction.FORWARD);
        glyphGrab.scaleRange( 20.0 / 255.0, 220.0 / 255.0 );
        glyphGrab.setPosition(glyphGrabPosition);

        glyphPush.setDirection(Servo.Direction.FORWARD);
        glyphPush.scaleRange( 110.0 / 255.0, 140.0 / 255.0 );
        glyphPush.setPosition(glyphPushPosition);

        jewelServoBlue.setDirection(Servo.Direction.REVERSE);
        jewelServoBlue.scaleRange( 67.0 / 255.0, 252.0 / 255.0 );
        jewelServoBlue.setPosition(JEWEL_UP);

        jewelServoRed.scaleRange( 58.0 / 255.0, 239.0 / 255.0 );
        jewelServoRed.setPosition(JEWEL_UP);

        relicExtend(0.0);
        relicElbow.setPower(RELIC_ELBOW_STOP);

        relicWrist.setDirection(Servo.Direction.FORWARD);
        relicWrist.scaleRange( 20.0 / 255.0, 200.0 / 255.0 );
        relicWrist.setPosition(RELIC_WRIST_MID);

        relicGrab.setDirection(Servo.Direction.REVERSE);
        relicGrab.scaleRange( 160.0 / 255.0, 230.0 / 255.0 );
        relicGrab.setPosition(relicGrabPosition);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);


        fr.setPower(0.);
        br.setPower(0.);
        fl.setPower(0.);
        bl.setPower(0.);

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


        Orientation angles;

    }

    public void autoInit(HardwareMap someHwMap){

        init(someHwMap);
        telemetry.addData("Vuforia", "Begining Vuforia Init");
        telemetry.update();
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.vuforiaLicenseKey = "ARYJT0b/////AAAAGYhN7cav+UUXqkMo7uS9Mswt0KxiQ3Sp/OVgoLfwHMP74uJpsnWLAXQLoXs0AIcpgC2IiJIov+JwDwrMwujShtlUastkjxWBAXLvJ6drxd811wEZGqBtBeOC6ObFPqG+W41u3D0fWJjsU4qG3S6NdgIAv6Q4T1OGH6Q6jOpatGlpEyhclM0Rk+vs77zaVzgBgZmcCa+tTqOpu0hhxqyxMvPv3Ehn0sgbF1KTfba/QQfxEjpsqJRyA5r7HfNNfg/31xdLLtzQXy28id0EXqPkB2iZ39fxsX0XcbKRWd7pq5uXqfvwJm4EvsKFLOz0eJhJBW+2vlCy5jrdehA7wH+pOnQTx3SQmbyqlr8KehWPWL1X";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        picReader = ClassFactory.createVuforiaLocalizer(params);
        VuforiaTrackables relicTrackables = picReader.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        telemetry.addData("Vuforia", "Ending Vuforia Init");
        telemetry.update();
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void teleInit(HardwareMap someHwMap){


        init(someHwMap);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //public double getHeading(){
        //Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //double angle = angles.firstAngle;
        //return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle));
   // }

public void setRunMode(DcMotor.RunMode runMode){
    bl.setMode(runMode);
    br.setMode(runMode);
    fl.setMode(runMode);
    fr.setMode(runMode);
    lift.setMode(runMode);
}

public void setDriveRunMode(DcMotor.RunMode runMode){
    bl.setMode(runMode);
    br.setMode(runMode);
    fl.setMode(runMode);
    fr.setMode(runMode);
}
public void stopRelic(){
    relicExtend(0.0);
    relicElbow.setPower(0.0);
}
    public void moveThatRobot(double speed, double inches, double timeout){
        moveThatRobot(speed, speed, speed, speed, inches, inches, inches, inches, timeout);
    }

    public void moveThatRobot(double rightSpeed, double leftSpeed, double inches, double timeout){
        moveThatRobot(rightSpeed, rightSpeed, leftSpeed, leftSpeed, inches, inches, inches, inches, timeout);
    }

    public void moveThatRobot(double rightSpeed, double leftSpeed, double rightInches, double leftInches, double timeout){
        moveThatRobot(rightSpeed, rightSpeed, leftSpeed, leftSpeed, rightInches, rightInches, leftInches, leftInches, timeout);
    }

    public void moveThatRobotSide(double frspeed, double brspeed, double flspeed, double blspeed, double inches, double timeout){
        moveThatRobot(frspeed, brspeed, flspeed, blspeed, inches, inches, inches, inches, timeout);
    }

    public void moveThatRobot(double frontRightSpeed, double backRightSpeed,
                              double frontLeftSpeed, double backLeftSpeed,
                              double frontRightInches, double backRightInches,
                              double frontLeftInches,  double backLeftInches,
                              double timeout){
        int newFrontRightTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newBackLeftTarget;
        double frCurrent;
        double brCurrent;
        double flCurrent;
        double blCurrent;
        double frComplete;
        double brComplete;
        double flComplete;
        double blComplete;
        double frSlowdown;
        double brSlowdown;
        double flSlowdown;
        double blSlowdown;

        frontRightSpeed = frontRightSpeed * (1.0 + 12.5/70.0);
        backRightSpeed = backRightSpeed * (1.0 + 12.5/70.0);

        //are we still running? good. if so:
        if (opModeIsActive()) {
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //now, where do we go? let's set the target position.
            newFrontRightTarget = fr.getCurrentPosition() + (int) (frontRightInches * Hardware.COUNTS_PER_INCH);
            newBackRightTarget = br.getCurrentPosition() + (int) (backRightInches * Hardware.COUNTS_PER_INCH);
            newFrontLeftTarget = fl.getCurrentPosition() + (int) (frontLeftInches * Hardware.COUNTS_PER_INCH);
            newBackLeftTarget = bl.getCurrentPosition() + (int) (backLeftInches * Hardware.COUNTS_PER_INCH);
            fr.setTargetPosition(newFrontRightTarget);
            br.setTargetPosition(newBackRightTarget);
            fl.setTargetPosition(newFrontLeftTarget);
            bl.setTargetPosition(newBackLeftTarget);

            //now you gotta make sure they know what to do with this info. give the motor a runmode.
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fr.setPower(Math.abs(frontRightSpeed));
            br.setPower(Math.abs(backRightSpeed));
            fl.setPower(Math.abs(frontLeftSpeed));
            bl.setPower(Math.abs(backLeftSpeed));
            double lastSpeedSetTime = runtime.seconds();

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (fr.isBusy() && br.isBusy() && fl.isBusy() && bl.isBusy())) {

                frCurrent = fr.getCurrentPosition();
                brCurrent = br.getCurrentPosition();
                flCurrent = fl.getCurrentPosition();
                blCurrent = bl.getCurrentPosition();


                frComplete = (double) frCurrent / (double) newFrontRightTarget;
                brComplete = (double) brCurrent / (double) newBackRightTarget;
                flComplete = (double) flCurrent / (double) newFrontLeftTarget;
                blComplete = (double) blCurrent / (double) newBackLeftTarget;

                frSlowdown = 1.0;
                if (frComplete > 0.92) {
                    frSlowdown = 0.5;
                } else if (frComplete > 0.85) {
                    frSlowdown = 0.8;
                }
                brSlowdown = 1.0;
                if (brComplete > 0.92) {
                    brSlowdown = 0.5;
                } else if (brComplete > 0.85) {
                    brSlowdown = 0.8;
                }
                flSlowdown = 1.0;
                if (flComplete > 0.92) {
                    flSlowdown = 0.5;
                } else if (flComplete > 0.85) {
                    flSlowdown = 0.8;
                }
                blSlowdown = 1.0;
                if (blComplete > 0.92) {
                    blSlowdown = 0.5;
                } else if (blComplete > 0.85) {
                    blSlowdown = 0.8;
                }

                if ((runtime.seconds() - lastSpeedSetTime) > 0.04) {
                    fr.setPower(frSlowdown * Math.abs(frontRightSpeed));
                    br.setPower(brSlowdown * Math.abs(backRightSpeed));
                    fl.setPower(flSlowdown * Math.abs(frontLeftSpeed));
                    bl.setPower(blSlowdown * Math.abs(backLeftSpeed));
                    lastSpeedSetTime = runtime.seconds();
                }

                opMode.idle();
            }

            // Stop all motion;
            fr.setPower(0.);
            br.setPower(0.);
            fl.setPower(0.);
            bl.setPower(0.);

            // Turn off RUN_TO_POSITION
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        }

    public void spinTurn(double degrees, double power){
        imu.initialize(parameters);
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double heading = getHeading();
        double currentHeading = heading;
        if(degrees <= 0) {
            fr.setPower(-power);
            br.setPower(-power);
            fl.setPower(power);
            bl.setPower(power);
            while(currentHeading - heading > degrees) {
                opMode.sleep(50);
                telemetry.addData("cool", "Current Heading: " + currentHeading);
                telemetry.addData("fun", "Original Angle : " + heading);
                telemetry.update();
                currentHeading = getHeading();
            }
            setAllPowers(0);
        }
        if(degrees >= 0){
            fr.setPower(power);
            br.setPower(power);
            fl.setPower(-power);
            bl.setPower(-power);
            while(currentHeading - heading < degrees){
                opMode.sleep(50);
                telemetry.addData("cool", "Current Heading: " + currentHeading);
                telemetry.addData("fun", "Original Angle : " + heading);
                telemetry.update();
                currentHeading = getHeading();
            }
            setAllPowers(0.0);
        }
    }

    public void swingTurn(double degrees, double power){
        imu.initialize(parameters);
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double heading = getHeading();
        double currentHeading = heading;
        if(degrees <= 0) {
            fr.setPower(0.0);
            br.setPower(0.0);
            fl.setPower(power);
            bl.setPower(power);
            while(currentHeading - heading > degrees) {
                opMode.sleep(50);
                telemetry.addData("cool", "Current Heading: " + currentHeading);
                telemetry.addData("fun", "Original Angle : " + heading);
                telemetry.update();
                currentHeading = getHeading();
            }
            setAllPowers(0);
        }
        if(degrees > 0){
            fr.setPower(power);
            br.setPower(power);
            fl.setPower(0.0);
            bl.setPower(0.0);
            while(currentHeading - heading < degrees){
                opMode.sleep(50);
                telemetry.addData("cool", "Current Heading: " + currentHeading);
                telemetry.addData("fun", "Original Angle : " + heading);
                telemetry.update();
                currentHeading = getHeading();
            }
            setAllPowers(0.0);
        }
    }

    public void setAllPowers(double power){
        fr.setPower(power * (1.0 +12.5/70.0));
        br.setPower(power * (1.0 +12.5/70.0));
        fl.setPower(power);
        bl.setPower(power);
    }

    private boolean opModeIsActive(){return opMode.opModeIsActive();}

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double angle = angles.firstAngle;
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle));
    }

    public void moveLift(int encoderCount){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (encoderCount > 0)
            encoderCount -= 200;
        lift.setTargetPosition(encoderCount);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.5);
        while(lift.isBusy()){
            opMode.idle();
        }
        if(encoderCount > 0){
            lift.setPower(0.0);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void relicExtend(double power)
    {
        relicExtendArm1.setPower(power);
        relicExtendArm2.setPower(power);
    }
}


