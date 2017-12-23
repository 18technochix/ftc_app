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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Techno Team_PC_III on 12/2/2017.
 */

@Disabled
public class Hardware{
    //public OpMode members
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
    CRServo relicWrist = null;
    Servo relicGrab = null;
    //sensors
    BNO055IMU imu;
    //ColorSensor tapeSensor1 = null;
    //ColorSensor tapeSensor2 = null;
    //VuforiaLocalizer picReader = null;
    MultiplexColorSensor colorSensor = null;
    //constants
    double GLYPH_GRAB_OPEN = 1.0;
    double GLYPH_GRAB_CLOSE = 0.0;
    double glyphGrabPosition = GLYPH_GRAB_CLOSE;

    double RELIC_GRAB_OPEN = 1.0;
    double RELIC_GRAB_CLOSE = 0.0;
    double relicGrabPosition = RELIC_GRAB_CLOSE;

    double RELIC_ELBOW_STOP = 0.0 - 2.0/255.;

    double JEWEL_DOWN = 0.0;
    double JEWEL_UP = 1.0;
    double redMin = 0;
    double redMax = 20;
    double blueMin = 150;
    double blueMax = 360;
    int bluePort = 2;
    int redPort = 1;
    static final int colorSampleMilliseconds = 48;

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
        jewelServoBlue = hwMap.get(Servo.class, "jewelServoBlue");
        jewelServoRed = hwMap.get(Servo.class, "jewelServoRed");
        relicExtendArm1 =hwMap.get(CRServo.class, "relicExtendArm1");
        relicExtendArm2 =hwMap.get(CRServo.class, "relicExtendArm2");
        relicElbow =hwMap.get(CRServo.class, "relicElbow");
        relicWrist=hwMap.get(CRServo.class, "relicWrist");
        relicGrab =hwMap.get(Servo.class, "relicGrab");
        //sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        int[] ports = {bluePort, redPort};
        colorSensor = new MultiplexColorSensor(someHwMap, "mux", "color sensor", ports, colorSampleMilliseconds,
                MultiplexColorSensor.GAIN_16X);
        //tapeSensor1 = hwMap.colorSensor.get("tapeSensor1");
        //tapeSensor2 = hwMap.colorSensor.get("tapeSensor2");
        //picReader = hwMap.get(VuforiaLocalizer.class, "picReader");

        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);

        glyphGrab.setDirection(Servo.Direction.FORWARD);
        glyphGrab.scaleRange(0.137, 0.863);
        glyphGrab.setPosition(glyphGrabPosition);

        jewelServoBlue.setDirection(Servo.Direction.REVERSE);
        jewelServoBlue.scaleRange(
                0.176, // 45
                0.843 // 215
                //0.823 // 210
                );
        jewelServoBlue.setPosition(JEWEL_UP);

        jewelServoRed.scaleRange(
                // 0.176 // 45
                0.196 // 50
                , 0.882 // 225
                );
        jewelServoRed.setPosition(JEWEL_UP);

        relicExtend(0.0);
        relicElbow.setPower(RELIC_ELBOW_STOP);
        relicWrist.setPower(0.0);

        relicGrab.setDirection(Servo.Direction.REVERSE);
        relicGrab.scaleRange(160.0/255.0, 230.0/255.0);
        relicGrab.setPosition(relicGrabPosition);

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);


        fr.setPower(0.);
        br.setPower(0.);
        fl.setPower(0.);
        bl.setPower(0.);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        /**int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = "ARYJT0b/////AAAAGYhN7cav+UUXqkMo7uS9Mswt0KxiQ3Sp/OVgoLfwHMP74uJpsnWLAXQLoXs0AIcpgC2IiJIov+JwDwrMwujShtlUastkjxWBAXLvJ6drxd811wEZGqBtBeOC6ObFPqG+W41u3D0fWJjsU4qG3S6NdgIAv6Q4T1OGH6Q6jOpatGlpEyhclM0Rk+vs77zaVzgBgZmcCa+tTqOpu0hhxqyxMvPv3Ehn0sgbF1KTfba/QQfxEjpsqJRyA5r7HfNNfg/31xdLLtzQXy28id0EXqPkB2iZ39fxsX0XcbKRWd7pq5uXqfvwJm4EvsKFLOz0eJhJBW+2vlCy5jrdehA7wH+pOnQTx3SQmbyqlr8KehWPWL1X";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.picReader = ClassFactory.createVuforiaLocalizer(params);
        VuforiaTrackables relicTrackables = this.picReader.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
         */


        Orientation angles;

    }

    public void autoInit(HardwareMap someHwMap){

        init(someHwMap);
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
}

    public void moveThatRobot(double speed, double inches, double timeout){
        moveThatRobot(speed, speed, speed, speed, inches, inches, inches, inches, timeout, "MOVE");
    }

    public void moveThatRobot(double frontRightSpeed, double backRightSpeed,
                              double frontLeftSpeed, double backLeftSpeed,
                              double frontRightInches, double backRightInches,
                              double frontLeftInches,  double backLeftInches,
                              double timeout, String tag){
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
                    (fr.isBusy() || br.isBusy() || fl.isBusy() || bl.isBusy())) {

                frCurrent = fr.getCurrentPosition();
                brCurrent = br.getCurrentPosition();
                flCurrent = fl.getCurrentPosition();
                blCurrent = bl.getCurrentPosition();

                // Display it for the driver.
                //telemetry.addData(tag + ": Path1", "Running to %7d :%7d", newFrontRightTarget, newBackRightTarget, newFrontLeftTarget, newBackLeftTarget);
                //telemetry.addData(tag + ": Path2", "Running at %7d :%7d", frCurrent, brCurrent, flCurrent, blCurrent);
                //telemetry.update();

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
    private boolean opModeIsActive(){return opMode.opModeIsActive();}

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double angle = angles.firstAngle;
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle));
    }

    public void relicExtend(double power)
    {
        relicExtendArm1.setPower(power);
        relicExtendArm2.setPower(power);
    }
}


