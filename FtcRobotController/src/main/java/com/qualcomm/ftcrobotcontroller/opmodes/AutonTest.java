package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Techno Team_PC_III on 11/22/2015.
 */
public class AutonTest extends LinearOpMode {

    // Wheel creation ///////////////////////

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





    // Init and Start /////////////////////////////////////////////////////////////////////////////

    public void runOpMode() throws InterruptedException{

        // Init //////////////////////////////////////

        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");

        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        aL = hardwareMap.dcMotor.get("aL");
        aR = hardwareMap.dcMotor.get("aR");

        aR.setDirection(DcMotor.Direction.REVERSE);

        left = hardwareMap.servo.get("sL");
        right = hardwareMap.servo.get("sR");

        beacon = hardwareMap.servo.get("beacon");

        fruity = hardwareMap.colorSensor.get("fruity");


        // init for servos /////////////////////

        left.setPosition(leftDown);
        right.setPosition(rightDown);

        beacon.setPosition(mid);

        resetEncoders();

        // Start /////////////////////////////////////

        waitForStart();


        if(opModeIsActive()){

          sense();
          // move(96);

        }


    } // End of Init and Start //////////////////////////////////////////////////////////////////







    // Directions ///////////////////////////////////////////////////////////////////////////////

    public void move(double distance) throws InterruptedException {

        move(distance, distance, distance, distance);

    }

    public void move(double dfL, double dfR, double dbL, double dbR) throws InterruptedException{

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


    // Servo / Adafruit //////////////////////////////////////////////////////////////////////////

    public void moveBeacon(double position) throws InterruptedException {

        beacon.setPosition(position);


        while(beacon.getPosition() > position + servorange ||
                beacon.getPosition() < position - servorange) {
            sleep(100);
            waitOneFullHardwareCycle();
        }



    }

    public void sense() throws InterruptedException { //THIS IS WRITTEN FOR THE RED ALLIANCE

        float hsvValues [] = {0F,0F,0F};

        moveBeacon(midLeft);
        waitOneFullHardwareCycle();

        sleep(500);

        int leftBlue = fruity.blue();
        int leftRed = fruity.red();
        int leftGreen = fruity.green();
        Color.RGBToHSV((leftRed * 255) / 800, (leftGreen * 255) / 800, (leftBlue * 255) / 800, hsvValues);
        float leftHue = hsvValues[0];
        waitOneFullHardwareCycle();

        sleep(500);

        moveBeacon(midRight);
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
        if(leftHue < hue && rightHue > hue){
            // RIGHT IS RED
            moveBeacon(fullRight);
            sleep(500);
            move(3);
            sleep(200);
            move(-3);
        } else if (leftHue > hue && rightHue < hue){
            //LEFT IS RED
            moveBeacon(fullLeft);
            sleep(500);
            move(3);
            sleep(200);
            move(-3);
        }


    }






    // Other Methods //////////////////////////////////////////////////////////////////////////////


    public void resetEncoders(){

        fR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        bR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        fL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        bL.setMode(DcMotorController.RunMode.RESET_ENCODERS);

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
