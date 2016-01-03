package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Techno Team_PC_III on 1/3/2016.
 */
public class RobotOpMode extends LinearOpMode{

    // true for auto, true for red, false for teleop, false for blue
    boolean auto;
    boolean red;

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


    // Flipper creation ////////////////////

    Servo leftFlipper;
    Servo rightFlipper;

    final double leftDown = 0.5; // value for left servo's extended position
    final double leftUp = 0.25; //value for left servo's retracted position

    final double rightDown = 0; //value for right servo's extended position
    final double rightUp = 0.25; //value for right servo's retracted position

    // Cowcatcher /////////////////////////////////////////////////////

    DcMotor leftPlow;
    DcMotor rightPlow;

    Servo cowLeft;
    Servo cowRight;
    double plowPower = 0.3;
    
    Servo plowTop;
    
    DcMotor plow;


    // Beacon servo creation ///////////////

    Servo beacon;

    double servorange = 0.05;

    double fullLeft = 0.0;
    double midLeft = 0.35;
    double mid = 0.5;
    double midRight = 0.65;
    double fullRight = 1.0;


    // Aidafruit creation ////////////////////////

    ColorSensor fruity;

    float hue = 300;
    //blue is less than, red greater

    // Sonar and Line Following //////////////////

    UltrasonicSensor eyes;
    double distance = 1000;

    LightSensor lightL;
    LightSensor lightR;
    double lightRight;
    double lightLeft;

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

        rightPlow.setDirection(DcMotor.Direction.REVERSE);

        leftFlipper = hardwareMap.servo.get("flipperl");
        rightFlipper = hardwareMap.servo.get("flipperr");
        
        plowTop = hardwareMap.servo.get("plowtop");

        //plow = hardwareMap.dcMotor.get("plow");

        cowLeft = hardwareMap.servo.get("cowleft");
        cowRight = hardwareMap.servo.get("cowright");

        beacon = hardwareMap.servo.get("beacon");

        fruity = hardwareMap.colorSensor.get("fruity");

        eyes = hardwareMap.ultrasonicSensor.get("eyes");

        lightR = hardwareMap.lightSensor.get("lineright");
        lightL = hardwareMap.lightSensor.get("lineleft");

        if(auto){

            // init for servos /////////////////////

            leftFlipper.setPosition(leftDown);
            rightFlipper.setPosition(rightDown);
            lightL.enableLed(true);
            lightR.enableLed(true);

            beacon.setPosition(mid);

            resetEncoders();

        }else{ // teleop configuration

            leftFlipper.setPosition(leftDown);
            rightFlipper.setPosition(rightDown);
            beacon.setPosition(fullRight);
        }



    }



    // Move Commands ///////////////////////////////////////////////////////////////////////////////

    public void move( double distance, double power) throws InterruptedException {

        move( distance, distance, distance, distance, power);

    }

    //positive is clockwise aka turning right
    public void pivot( double distance, double power) throws InterruptedException {

        move( distance, -distance, distance, -distance, power);

    }



    public void move( double dfL, double dfR, double dbL, double dbR, double power) throws InterruptedException{

        //resetEncoders();

        getNewPositions();

        fR.setTargetPosition(currentFR + counts(dfR));
        fL.setTargetPosition(currentFL + counts(dfL));
        bR.setTargetPosition(currentBR + counts(dbR));
        bL.setTargetPosition(currentBL + counts(dbL));

        fR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        fL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        fR.setPower(lopower);
        bR.setPower(lopower);
        fL.setPower(lopower);
        bL.setPower(lopower);

        while(!atPosition((currentBL + counts(dbL)), (currentBR + counts(dbR)),
                (currentFL + counts(dfL)),(currentFR + counts(dfR))) ){

            telemetry.addData("Pos:", String.format("%03d %03d %03d %03d", bR.getCurrentPosition(),
                    fR.getCurrentPosition(), bL.getCurrentPosition(), bR.getCurrentPosition()));

            waitOneFullHardwareCycle();

        }

        fR.setPower(0.0);
        bR.setPower(0.0);
        fL.setPower(0.0);
        bL.setPower(0.0);

        waitOneFullHardwareCycle();


    }

    public void startWheels(double power){

        fR.setPower(power);
        bR.setPower(power);
        fL.setPower(power);
        bL.setPower(power);

    }

    public void startWheels(double powerL, double powerR){

        fR.setPower(powerR);
        bR.setPower(powerR);
        fL.setPower(powerL);
        bL.setPower(powerL);

    }

    public void stopWheels(){

        fR.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        bL.setPower(0);

    }


    public void moveBeacon( double position) throws InterruptedException {

        beacon.setPosition(position);


        while(beacon.getPosition() > position + servorange ||
                beacon.getPosition() < position - servorange) {
            sleep(100);
            waitOneFullHardwareCycle();
        }



    }

    public boolean followLine() throws InterruptedException {

        runWithoutEncoders();

        distance = eyes.getUltrasonicLevel();

        if(distance > 20.0) {

            telemetry.addData("Floor reading", String.format("%.4f %.4f", lightLeft, lightRight));
            telemetry.addData("Distance", distance);

            lightLeft = lightL.getLightDetected();
            lightRight = lightR.getLightDetected();
            distance = eyes.getUltrasonicLevel();

            if(lightLeft > 0.8){

                double startTime = getRuntime();
                startWheels(0.1, -0.1);
                waitOneFullHardwareCycle();
                while(getRuntime() < startTime + 0.25){
                    waitOneFullHardwareCycle();
                }
                stopWheels();
                waitOneFullHardwareCycle();

            }else if(lightRight > 0.8){

                double startTime = getRuntime();
                startWheels(-0.1, 0.1);
                waitOneFullHardwareCycle();
                while(getRuntime() < startTime + 0.25){
                    waitOneFullHardwareCycle();
                }
                stopWheels();
                waitOneFullHardwareCycle();

            } else{

                move(1, slowpower);

            }

            return false;

        }else {
            return true;
        }

    }

    public void sense() throws InterruptedException { //THIS IS WRITTEN FOR THE RED ALLIANCE

        float hsvValues [] = {0F,0F,0F};

        moveBeacon( midLeft);
        waitOneFullHardwareCycle();

        sleep(500);

        int leftBlue = fruity.blue();
        int leftRed = fruity.red();
        int leftGreen = fruity.green();
        Color.RGBToHSV((leftRed * 255) / 800, (leftGreen * 255) / 800, (leftBlue * 255) / 800, hsvValues);
        float leftHue = hsvValues[0];
        waitOneFullHardwareCycle();

        sleep(500);

        moveBeacon( midRight);
        waitOneFullHardwareCycle();

        sleep(3000);
        waitOneFullHardwareCycle();

        hsvValues [0] = 0;

        int rightRed = fruity.red();
        int rightBlue = fruity.blue();
        int rightGreen = fruity.green();
        Color.RGBToHSV((rightRed * 255) / 800, (rightGreen * 255) / 800, (rightBlue * 255) / 800, hsvValues);
        float rightHue = hsvValues[0];
        waitOneFullHardwareCycle();

        telemetry.addData("left", String.format("%03d %03d %03d", leftRed, leftGreen, leftBlue));
        telemetry.addData("right", String.format("%03d %03d %03d", rightRed, rightGreen, rightBlue));
        telemetry.addData("leftHue", leftHue);
        telemetry.addData("rightHue", rightHue);

        sleep(500);

        if(red) {

            if (leftHue < hue && rightHue > hue) {
                // RIGHT IS RED
                moveBeacon( fullRight);
                sleep(500);
                move( 3, hipower);
                sleep(200);
                move( -3, hipower);
            } else if (leftHue > hue && rightHue < hue) {
                //LEFT IS RED
                moveBeacon(fullLeft);
                sleep(500);
                move( 3, hipower);
                sleep(200);
                move( -3.0, hipower);
            }

        }else{

            if (leftHue < hue && rightHue > hue) {
                // RIGHT IS RED
                moveBeacon( fullLeft);
                sleep(500);
                move( 3, hipower);
                sleep(200);
                move( -3, hipower);
            } else if (leftHue > hue && rightHue < hue) {
                //LEFT IS RED
                moveBeacon( fullRight);
                sleep(500);
                move( 3, hipower);
                sleep(200);
                move( -3.0, hipower);
            }

        }


    }


    public void teleop() throws InterruptedException {

        // Controls for the wheels

        bL.setPower(scaleInput(gamepad1.left_stick_y));
        fL.setPower(scaleInput(gamepad1.left_stick_y));

        bR.setPower(scaleInput(gamepad1.right_stick_y));
        fR.setPower(scaleInput(gamepad1.right_stick_y));

        //Controls for the servos
        // The dpad controls the left side, the buttons control the right

        if(gamepad2.dpad_left){ //if left dpad is pressed, move left servo to the outside position

            leftFlipper.setPosition(leftUp);

        }else if(gamepad2.dpad_right){ //if the right dpad is pressed, move left servo to inward position

            leftFlipper.setPosition(leftDown);

        }

        if(gamepad2.b){

            rightFlipper.setPosition(rightUp);

        }else if(gamepad2.x){

            rightFlipper.setPosition(rightDown);

        }

        if(gamepad2.dpad_down){
            leftPlow.setTargetPosition(0);
            leftPlow.setPower(plowPower);
            rightPlow.setTargetPosition(0);
            rightPlow.setPower(plowPower);
        }else if(gamepad2.dpad_up){
            leftPlow.setTargetPosition(500);
            leftPlow.setPower(plowPower);
            rightPlow.setTargetPosition(500);
            rightPlow.setPower(plowPower);
        }
        else{
            leftPlow.setPower(0.);
            rightPlow.setPower(0.);
        }

        /*
        if(gamepad2.a){

        }else if(gamepad2.y){

        }
        else{

        }
        */


    }

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




    // Other Commands //////////////////////////////////////////////////////////////////////////////


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



}
