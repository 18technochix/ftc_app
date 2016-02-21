package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;
import android.util.Log;

import com.github.ImperialRobotics.BoschIMU.AdafruitIMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * Created by Techno Team_PC_III on 1/3/2016.
 */
public class RobotOpMode extends LinearOpMode{

    /*
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //Object creation
    ////////////////////////////////////////////////////////////////////////////////////////////////
     */

    // true for auto, true for red, false for teleop, false for blue
    boolean auto;
    boolean red;

    // Wheels //////////////////////////////////////////////////////////////////////////////////////

    DcMotor fR;
    DcMotor bR;
    DcMotor fL;
    DcMotor bL;

    int currentBL = 0;
    int currentBR = 0;
    int currentFR = 0;
    int currentFL = 0;

    final int magicnum = 1120; //1440 for tetrix motors, 1120 for andymark40
    final double diameter = 5;
    final double circ = Math.PI * diameter;
    final double ratio = 1;

    double hipower = 0.7;
    double lopower = 0.3;

    int range = 15; // range for encoders?

    // Dispenser Arms //////////////////////////////////////////////////////////////////////////////

    DcMotor dispL;
    DcMotor dispR;

    final double dispPower = 0.3;
    final int dispPosition = 5500;
    final int climbersPosition = 3000;

    // Dispenser Bucket ////////////////////////////////////////////////////////////////////////////

    Servo release; // this is the two(using a y-connector) that relase the debris from the bucket
    double releaseClosed = 0.9;
    double releaseOpen = 0.4 ;

    Servo tilt; // used to tilt the bucket, needs to be variable
    double tiltMiddle = 0.5;

    Servo climbers; // releases the climbers
    double climbersClosed = 0.45;
    double climbersOpen = 0.75;

    // Cowcatcher //////////////////////////////////////////////////////////////////////////////////

    Servo cowLeft; //closed is 0, open is 1
    Servo cowRight; //closed is 1, open is 0

    double cowLeftOpen =  0.35;
    double cowRightOpen = 0.75;

    // Harvester //////////////////////////////////////////////////////////////////////////////////

    DcMotor harvester;

    // Beacon servo ////////////////////////////////////////////////////////////////////////////////

    Servo beacon;


    double servorange = 0.05;

    double fullLeft = 0.0;
    double midLeft = 0.35;
    double mid = 0.5;
    double midRight = 0.65;
    double fullRight = 1.0;


    // Color Sensor ////////////////////////////////////////////////////////////////////////////////////

    ColorSensor fruity;

    float hue = 300;
    //blue is less than, red greater

    // Gyro ////////////////////////////////////////////////////////////////////////////////////////

    AdafruitIMU gyro;

    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    // Line Following ///////////////////////////////////////////////////////////////////

    LightSensor lightL;
    LightSensor lightR;
    double lightRight;
    double lightLeft;

    // Touch sensor ////////////////////////////////////////////////////////////////////////////////

    TouchSensor touchy;

    /*

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //Init Sequence
    ////////////////////////////////////////////////////////////////////////////////////////////////

     */


    public void runOpMode() throws InterruptedException{

        initialize();
        waitOneFullHardwareCycle();
        runDispensersToPosition();
        waitOneFullHardwareCycle();

    }

    public void initialize() throws InterruptedException {

        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");

        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        dispL = hardwareMap.dcMotor.get("dL");
        dispR = hardwareMap.dcMotor.get("dR");

        dispR.setDirection(DcMotor.Direction.REVERSE);

        resetDispenserEncoders();

        cowLeft = hardwareMap.servo.get("cowleft");
        cowRight = hardwareMap.servo.get("cowright");

        cowLeft.setPosition(0);
        cowRight.setPosition(1);

        harvester = hardwareMap.dcMotor.get("harvest");

        beacon = hardwareMap.servo.get("beacon");

        fruity = hardwareMap.colorSensor.get("fruity");

        lightR = hardwareMap.lightSensor.get("lineright");
        lightL = hardwareMap.lightSensor.get("lineleft");

        touchy = hardwareMap.touchSensor.get("touchy");

        release = hardwareMap.servo.get("release");
        tilt = hardwareMap.servo.get("tilt");
        climbers = hardwareMap.servo.get("climbers");

        release.setPosition(releaseClosed);
        tilt.setPosition(tiltMiddle);
        climbers.setPosition(climbersClosed);



        try {
            gyro = new AdafruitIMU(hardwareMap, "gyro"
                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)
                    , (byte)AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
        }



        if(auto){

            lightL.enableLed(true);
            lightR.enableLed(true);

            beacon.setPosition(fullLeft);

            release.setPosition(releaseClosed);

            gyro.startIMU();

        }else{ // teleop configuration
            beacon.setPosition(fullRight);
            runWheelsWithoutEncoders();

        }




    }

    /*
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

     */

    // Encoders  ///////////////////////////////////////////////////////////////////////////////////

    public void resetWheelEncoders() throws InterruptedException {

        fR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        bR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        fL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        bL.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        for (int i = 0; i < 2; i++)
            waitOneFullHardwareCycle();

    }

    public void runWheelsToPosition() throws InterruptedException {

        fR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        fL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < 2; i++)
            waitOneFullHardwareCycle();

    }


    public void getNewWheelPositions(){

        currentBL = bL.getCurrentPosition();
        currentBR = bR.getCurrentPosition();
        currentFL = fL.getCurrentPosition();
        currentFR = fR.getCurrentPosition();

    }

    public void runWheelsWithoutEncoders() throws InterruptedException {

        fL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        fR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        bL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        bR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        for (int i = 0; i < 2; i++)
            waitOneFullHardwareCycle();

    }


    // Dispenser Arm Encoders //////////////////////////////////////////////////////////////////////

    public void runDispensersToPosition() throws InterruptedException {

        dispL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        dispR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        waitOneFullHardwareCycle();

    }

    public void runDispenserWithoutEncoders() throws InterruptedException {

        dispL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        dispR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        waitOneFullHardwareCycle();

    }

    public void resetDispenserEncoders() throws InterruptedException {

        dispL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        dispR.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        waitOneFullHardwareCycle();

    }


    // Autonomous //////////////////////////////////////////////////////////////////////////////////

    // Wheel Power /////////////////////////////////////////////////////////////////////////////////

    public void startWheels(double power)throws InterruptedException {

        fR.setPower(power);
        bR.setPower(power);
        fL.setPower(power);
        bL.setPower(power);

    }

    public void stopWheels()throws InterruptedException {

        fR.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        bL.setPower(0);

    }

    public void turnLeft(double power){

        fR.setPower(-power);
        bR.setPower(-power);
        fL.setPower(power);
        bL.setPower(power);

    }

    public void turnRight(double power){

        fR.setPower(power);
        bR.setPower(power);
        fL.setPower(-power);
        bL.setPower(-power);

    }

    public void moveBeacon(double position) throws InterruptedException {

        beacon.setPosition(position);

        while(beacon.getPosition() > position + servorange ||
                beacon.getPosition() < position - servorange) {
            sleep(100);
            waitOneFullHardwareCycle();
        }


    }


    // IMU /////////////////////////////////////////////////////////////////////////////////////////

    public void printYawData(){

        /*

            This returns the yaw values of the IMU unit to show how far we have moved. It's
            printing back to the Driver Station.

         */

        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f",
                        yawAngle[0], yawAngle[1]));


    }

    public void getAngles(){

        /*

            This gathers new data from the IMU and stores it into variables.

         */

        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

    }

    public void refreshIMU(){

        getAngles();
        printYawData();

    }


    // Encoder Math ////////////////////////////////////////////////////////////////////////////////

    public boolean atWheelPosition(int target){

        if(inRange(fL.getCurrentPosition(), target)){ //RIGHT NOW IT'S SET TO THE FRONT LEFT
            return true;
        }

        return false;
    }

    public boolean atWheelPosition(int targetbL, int targetbR, int targetfL, int targetfR){

        if( inRange( bL.getCurrentPosition(), targetbL) &&
                inRange( bR.getCurrentPosition(), targetbR) &&
                inRange( fL.getCurrentPosition(), targetfL) &&
                inRange(fR.getCurrentPosition(), targetfR)){

            return true;

        }else{

            return false;

        }



    }

    public int counts(double distance){

        double rotations = distance / circ;
        return (int) (magicnum * rotations * ratio);

    }


    // Math ////////////////////////////////////////////////////////////////////////////////////////

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


    public boolean inRange(int value, int target){

        return (value >= target - range && value <= target + range);

    }




}
