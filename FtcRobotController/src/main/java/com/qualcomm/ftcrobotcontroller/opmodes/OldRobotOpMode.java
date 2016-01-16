package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Techno Team_PC_III on 1/3/2016.
 */
public class OldRobotOpMode extends LinearOpMode{

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
    final double diameter = 97.6/25.4;
    final double circ = Math.PI * diameter;
    final double ratio = 1;

    int range = 15;

    double hipower = 0.7;
    double lopower = 0.3;
    double slowpower = 0.1;

    double linePower = 0.12; // power for the wheels in follow line sequence
    double lineTime = 0.25; // time for wheels running in follow line sequence


    // Flipper creation ////////////////////////////////////////////////////////////////////////////

    Servo leftFlipper;
    Servo rightFlipper;

    final double leftDown = 0.5; // value for left servo's extended position
    final double leftUp = 0.25; //value for left servo's retracted position

    final double rightDown = 0; //value for right servo's extended position
    final double rightUp = 0.25; //value for right servo's retracted position

    // Cowcatcher //////////////////////////////////////////////////////////////////////////////////

    DcMotor leftPlow;
    DcMotor rightPlow;

    double armPower = 0.2;
    double autoArmPower = 0.4;
    int armFloor = 2700;
    int armClimbers = 1400;

    Servo cowLeft; //closed is 0, open is 1
    Servo cowRight; //closed is 1, open is 0

    double cowLeftOpen =  0.35;
    double cowRightOpen = 0.75;

    
    Servo plowTop; //1 is fully open, 0 is closed
    
    DcMotor plow;
    double plowPower = 0.1;
    boolean negPlow = false;


    // Beacon servo ////////////////////////////////////////////////////////////////////////////////

    Servo beacon;

    double servorange = 0.05;

    double fullLeft = 0.0;
    double midLeft = 0.35;
    double mid = 0.5;
    double midRight = 0.65;
    double fullRight = 1.0;


    // Adafruit ////////////////////////////////////////////////////////////////////////////////////

    ColorSensor fruity;

    float hue = 300;
    //blue is less than, red greater

    // Sonar and Line Following ///////////////////////////////////////////////////////////////////

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

    }

    public void initialize(){

        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");

        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        leftPlow = hardwareMap.dcMotor.get("leftplow");
        rightPlow = hardwareMap.dcMotor.get("rightplow");

        leftPlow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightPlow.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        leftPlow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightPlow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        leftPlow.setDirection(DcMotor.Direction.REVERSE);

        leftFlipper = hardwareMap.servo.get("flipperl");
        rightFlipper = hardwareMap.servo.get("flipperr");

        leftFlipper.setPosition(leftDown);
        rightFlipper.setPosition(rightDown);
        
        plowTop = hardwareMap.servo.get("plowtop");
        plowTop.setPosition(0); ///!!!!

        plow = hardwareMap.dcMotor.get("plow");
        plow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        plow.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        cowLeft = hardwareMap.servo.get("cowleft");
        cowRight = hardwareMap.servo.get("cowright");

        cowLeft.setPosition(0);
        cowRight.setPosition(1);

        beacon = hardwareMap.servo.get("beacon");

        fruity = hardwareMap.colorSensor.get("fruity");

        lightR = hardwareMap.lightSensor.get("lineright");
        lightL = hardwareMap.lightSensor.get("lineleft");

        touchy = hardwareMap.touchSensor.get("touchy");

        if(auto){

            // init for servos /////////////////////
            lightL.enableLed(true);
            lightR.enableLed(true);
            beacon.setPosition(fullLeft);
            resetEncoders();

        }else{ // teleop configuration
            beacon.setPosition(fullRight);
            runWithoutEncoders();
        }



    }

    /*
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

     */

    // Encoders  ///////////////////////////////////////////////////////////////////////////////////

    public void resetEncoders(){

        fR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        bR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        fL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        bL.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        leftPlow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightPlow.setMode(DcMotorController.RunMode.RESET_ENCODERS);

    }


    public void getNewPositions(){

        currentBL = bL.getCurrentPosition();
        currentBR = bR.getCurrentPosition();
        currentFL = fL.getCurrentPosition();
        currentFR = fR.getCurrentPosition();

    }

    public int counts(double distance){

        double rotations = distance / circ;
        return (int) (magicnum * rotations * ratio);

    }

    public boolean inRange(int value, int target){

        return (value >= target - range && value <= target + range);

    }


    public boolean atPosition(int targetbL, int targetbR, int targetfL, int targetfR){

        if( inRange( bL.getCurrentPosition(), targetbL) &&
                inRange( bR.getCurrentPosition(), targetbR) &&
                inRange( fL.getCurrentPosition(), targetfL) &&
                inRange(fR.getCurrentPosition(), targetfR)){

            return true;

        }else{

            return false;

        }

    }

    public void runWithoutEncoders(){

        fL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        fR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        bL.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        bR.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

    }

    // Wheel Power /////////////////////////////////////////////////////////////////////////////////

    public void startWheels(double power)throws InterruptedException {

        fR.setPower(power);
        bR.setPower(power);
        fL.setPower(power);
        bL.setPower(power);
        waitOneFullHardwareCycle();

    }

    public void startWheels(double powerL, double powerR)throws InterruptedException {

        fR.setPower(powerR);
        bR.setPower(powerR);
        fL.setPower(powerL);
        bL.setPower(powerL);
        waitOneFullHardwareCycle();

    }

    public void stopWheels()throws InterruptedException {

        fR.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        bL.setPower(0);
        waitOneFullHardwareCycle();

    }

    // Autonomous //////////////////////////////////////////////////////////////////////////////////

    public void moveBeacon( double position) throws InterruptedException {

        beacon.setPosition(position);

        while(beacon.getPosition() > position + servorange ||
                beacon.getPosition() < position - servorange) {
            sleep(100);
            waitOneFullHardwareCycle();
        }


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





}
