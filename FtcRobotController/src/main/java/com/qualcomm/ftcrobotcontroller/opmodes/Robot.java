package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Techno Team_PC_III on 1/3/2016.
 */
public class Robot{

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

    int range = 5;

    double hipower = 0.7;
    double lopower = 0.3;


    // Arm creation ////////////////////////

    DcMotor aL;
    DcMotor aR;

    // Flipper creation ////////////////////

    Servo left;
    Servo right;

    final double leftDown = 0.5; // value for left servo's extended position
    final double leftUp = 0.25; //value for left servo's retracted position

    final double rightDown = 0; //value for right servo's extended position
    final double rightUp = 0.25; //value for right servo's retracted position

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




    public Robot(boolean autonomous, boolean color){
        auto = autonomous;
        red = color;
    }




    public void initialize(LinearOpMode opmode){

        fR = opmode.hardwareMap.dcMotor.get("fR");
        bR = opmode.hardwareMap.dcMotor.get("bR");
        fL = opmode.hardwareMap.dcMotor.get("fL");
        bL = opmode.hardwareMap.dcMotor.get("bL");

        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        aL = opmode.hardwareMap.dcMotor.get("aL");
        aR = opmode.hardwareMap.dcMotor.get("aR");

        aR.setDirection(DcMotor.Direction.REVERSE);

        left = opmode.hardwareMap.servo.get("sL");
        right = opmode.hardwareMap.servo.get("sR");

        beacon = opmode.hardwareMap.servo.get("beacon");

        fruity = opmode.hardwareMap.colorSensor.get("fruity");

        if(auto){

            // init for servos /////////////////////

            left.setPosition(leftDown);
            right.setPosition(rightDown);

            beacon.setPosition(mid);

            resetEncoders();

        }else{ // teleop configuration

            left.setPosition(leftDown);
            right.setPosition(rightDown);
        }



    }



    // Move Commands ///////////////////////////////////////////////////////////////////////////////

    public void move(LinearOpMode opmode, double distance) throws InterruptedException {

        move(opmode, distance, distance, distance, distance);

    }

    public void move(LinearOpMode opmode, double dfL, double dfR, double dbL, double dbR) throws InterruptedException{

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

            opmode.telemetry.addData("Pos:", String.format("%03d %03d %03d %03d", bR.getCurrentPosition(),
                    fR.getCurrentPosition(), bL.getCurrentPosition(), bR.getCurrentPosition()));

            opmode.waitOneFullHardwareCycle();

        }

        fR.setPower(0.0);
        bR.setPower(0.0);
        fL.setPower(0.0);
        bL.setPower(0.0);

        opmode.waitOneFullHardwareCycle();


    }

    public void moveBeacon(LinearOpMode opmode, double position) throws InterruptedException {

        beacon.setPosition(position);


        while(beacon.getPosition() > position + servorange ||
                beacon.getPosition() < position - servorange) {
            opmode.sleep(100);
            opmode.waitOneFullHardwareCycle();
        }



    }

    public void sense(LinearOpMode opmode) throws InterruptedException { //THIS IS WRITTEN FOR THE RED ALLIANCE

        float hsvValues [] = {0F,0F,0F};

        moveBeacon(opmode, midLeft);
        opmode.waitOneFullHardwareCycle();

        opmode.sleep(500);

        int leftBlue = fruity.blue();
        int leftRed = fruity.red();
        int leftGreen = fruity.green();
        Color.RGBToHSV((leftRed * 255) / 800, (leftGreen * 255) / 800, (leftBlue * 255) / 800, hsvValues);
        float leftHue = hsvValues[0];
        opmode.waitOneFullHardwareCycle();

        opmode.sleep(500);

        moveBeacon(opmode, midRight);
        opmode.waitOneFullHardwareCycle();

        opmode.sleep(3000);
        opmode.waitOneFullHardwareCycle();

        hsvValues [0] = 0;

        int rightRed = fruity.red();
        int rightBlue = fruity.blue();
        int rightGreen = fruity.green();
        Color.RGBToHSV((rightRed * 255) / 800, (rightGreen * 255) / 800, (rightBlue * 255) / 800, hsvValues);
        float rightHue = hsvValues[0];
        opmode.waitOneFullHardwareCycle();

        opmode.telemetry.addData("left", String.format("%03d %03d %03d", leftRed, leftGreen, leftBlue));
        opmode.telemetry.addData("right", String.format("%03d %03d %03d", rightRed, rightGreen, rightBlue));
        opmode.telemetry.addData("leftHue", leftHue);
        opmode.telemetry.addData("rightHue", rightHue);

        opmode.sleep(500);

        if(red) {

            if (leftHue < hue && rightHue > hue) {
                // RIGHT IS RED
                moveBeacon(opmode, fullRight);
                opmode.sleep(500);
                move(opmode, 3);
                opmode.sleep(200);
                move(opmode, -3);
            } else if (leftHue > hue && rightHue < hue) {
                //LEFT IS RED
                moveBeacon(opmode,fullLeft);
                opmode.sleep(500);
                move(opmode, 3);
                opmode.sleep(200);
                move(opmode, -3.0);
            }

        } else{

            if (leftHue < hue && rightHue > hue) {
                // RIGHT IS RED
                moveBeacon(opmode, fullLeft);
                opmode.sleep(500);
                move(opmode, 3);
                opmode.sleep(200);
                move(opmode, -3);
            } else if (leftHue > hue && rightHue < hue) {
                //LEFT IS RED
                moveBeacon(opmode, fullRight);
                opmode.sleep(500);
                move(opmode, 3);
                opmode.sleep(200);
                move(opmode, -3.0);
            }

        }


    }


    public void teleop(LinearOpMode opmode) throws InterruptedException {

        // Controls for the wheels

        bL.setPower(scaleInput(opmode.gamepad1.left_stick_y));
        fL.setPower(scaleInput(opmode.gamepad1.left_stick_y));

        bR.setPower(scaleInput(opmode.gamepad1.right_stick_y));
        fR.setPower(scaleInput(opmode.gamepad1.right_stick_y));

        //Controls for the servos
        // The dpad controls the left side, the buttons control the right

        if(opmode.gamepad2.dpad_left){ //if left dpad is pressed, move left servo to the outside position

            left.setPosition(leftUp);

        }else if(opmode.gamepad2.dpad_right){ //if the right dpad is pressed, move left servo to inward position

            left.setPosition(leftDown);

        }

        if(opmode.gamepad2.b){

            right.setPosition(rightUp);

        }else if(opmode.gamepad2.x){

            right.setPosition(rightDown);

        }

        if(opmode.gamepad2.dpad_down){
            aL.setTargetPosition(0);
            aL.setPower(0.15);
        }else if(opmode.gamepad2.dpad_up){
            aL.setTargetPosition(500);
            aL.setPower(0.15);
        }
        else{
            aL.setPower(0.);
        }

        if(opmode.gamepad2.a){
            aR.setTargetPosition(0);
            aR.setPower(0.15);
        }else if(opmode.gamepad2.y){
            aR.setTargetPosition(500);
            aR.setPower(0.15);
        }
        else{
            aR.setPower(0.);
        }


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

        aL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        aR.setMode(DcMotorController.RunMode.RESET_ENCODERS);

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



}
