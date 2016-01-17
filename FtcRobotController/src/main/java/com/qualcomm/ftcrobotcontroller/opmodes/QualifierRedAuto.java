package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;


public class QualifierRedAuto extends QualifierRobotOpMode {
    double startTime;

    public void runOpMode() throws InterruptedException {

        auto = true;
        red = true;
        super.runOpMode(); //initalizes robot

        waitForStart();

        if (opModeIsActive()) {

            cowLeft.setPosition(cowLeftOpen);
            cowRight.setPosition(cowRightOpen);

            forward(1, 0.3);
            //move(24, 0.5); //move forward from the wall 15 inches
            turn(0.8, 0.7, true);// .57,
            forward (2.75, 0.3); // move(76, 0.5);
            turn(0.7, 0.7, true);

            while (followLine() == false) {

            }

            if(!sense()){

                startWheels(-linePower);
                waitForTime(0.65);
                stopWheels();

                startWheels(linePower);
                waitForTime(0.73);
                stopWheels();

                sense();

            }

            climbers();

        }
    }

    /*

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

     */


    public boolean sense() throws InterruptedException {

        startWheels(-0.1);
        waitForTime(0.25);
        stopWheels();

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
                moveBeacon(fullRight);
                sleep(500);
                move(3, hipower);
                sleep(200);
                move( -3, hipower);
                return true;

            } else if (leftHue > hue && rightHue < hue) {
                //LEFT IS RED
                moveBeacon(fullLeft);
                sleep(500);
                move(3, hipower);
                sleep(200);
                move( -3.0, hipower);
                return true;
            }



        }else{

            if (leftHue < hue && rightHue > hue) {
                // RIGHT IS RED
                moveBeacon(fullLeft);
                sleep(500);
                move(3, hipower);
                sleep(200);
                move( -3, hipower);
                return true;
            } else if (leftHue > hue && rightHue < hue) {
                //LEFT IS RED
                moveBeacon(fullRight);
                sleep(500);
                move(3, hipower);
                sleep(200);
                move( -3.0, hipower);
                return true;
            }



        }
        return false;


    }

    public boolean followLine() throws InterruptedException {

        beacon.setPosition(mid);
        //open beacon sensor

        //close cows
        cowLeft.setPosition(0);
        cowRight.setPosition(1);

        runWithoutEncoders();

        //distance = eyes.getUltrasonicLevel();

        //distance >= 20.0
        if(touchy.isPressed() == false) {

            telemetry.addData("Floor reading", String.format("%.4f %.4f", lightLeft, lightRight));

            lightLeft = lightL.getLightDetected();
            lightRight = lightR.getLightDetected();
            //distance = eyes.getUltrasonicLevel();

            /*
            if(lightLeft < 0.1 || lightRight < 0.1){ //if its red, move forward a bit

                startWheels(linePower);
                waitForTime(lineTime);
                stopWheels();


            } else
            */

            if(lightLeft > 0.65 && lightRight < 0.33){

                turn(0.15, 0.4, false);
                startWheels(linePower);
                waitForTime(lineTime);
                turn(0.25, 0.4, true);
                waitOneFullHardwareCycle();

            } else if(lightRight > 0.75 && lightLeft < 0.3){

                turn(0.15, 0.4, true);
                startWheels(linePower);
                waitForTime(lineTime);
                turn(0.25, 0.4, false);
                waitOneFullHardwareCycle();

            } else{
                startWheels(linePower);
                waitForTime(lineTime);
                stopWheels();

            }

            return false;

        }else {

            return true;

        }

    }

    public void forward(double seconds, double power) throws InterruptedException {

        runWithoutEncoders();

        startWheels(power, power);

        waitForTime(seconds);

        stopWheels();

    }

    public void turn(double seconds, double power, boolean left) throws InterruptedException {

        runWithoutEncoders();

        if(left){

            startWheels(-power, power);

        }else {

            startWheels(power, -power);

        }

        waitForTime(seconds);

        stopWheels();

    }

    // Climbers ////////////////////////////////////////////////////////////////////////////////////

    public void climbers() throws InterruptedException {

        leftPlow.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightPlow.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        leftPlow.setTargetPosition(armClimbers);
        leftPlow.setPower(autoArmPower);
        rightPlow.setTargetPosition(armClimbers);
        rightPlow.setPower(autoArmPower);

        while(leftPlow.getCurrentPosition() < armClimbers || rightPlow.getCurrentPosition() < armClimbers){

            waitOneFullHardwareCycle();

        }

        leftPlow.setTargetPosition(0);
        leftPlow.setPower(-autoArmPower);
        rightPlow.setTargetPosition(0);
        rightPlow.setPower(-autoArmPower);

        while(leftPlow.getCurrentPosition() > 0 || rightPlow.getCurrentPosition() > 0){

            waitOneFullHardwareCycle();

        }

        leftPlow.setPower(0);
        rightPlow.setPower(0);

    } //end of climbers void

    //Generic Methods //////////////////////////////////////////////////////////////////////////////

    public void waitForTime(double seconds) throws InterruptedException{
        startTime = getRuntime();
        while(getRuntime() < startTime + seconds){
            waitOneFullHardwareCycle();
        }
    } //end of wait for time void

    public void move(double distance, double power) throws InterruptedException {

        move(distance, distance, distance, distance, power);

    } // end of move (2var)

    //positive is clockwise aka turning right
    public void pivot( double distance, double power) throws InterruptedException {

        move( distance, -distance, distance, -distance, power);

    } //end of pivot

    public void move(double dfL, double dfR, double dbL, double dbR, double power)
            throws InterruptedException{

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


    } //end of move

}// end of auto class



