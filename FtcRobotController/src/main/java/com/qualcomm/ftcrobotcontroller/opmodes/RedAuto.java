package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Techno Team_PC_III on 11/22/2015.
 */
public class RedAuto extends RobotOpMode {
    double startTime;

    public void runOpMode() throws InterruptedException {



        auto = true;
        red = true;
        super.runOpMode(); //initalizes robot

        waitForStart();

        if (opModeIsActive()) {



            //robot.move(this, 96);

            beacon.setPosition(mid);

            while (followLine() == false) {

            }
            if(!sense()){

                startWheels(-linePower);
                waitForTime(0.25);
                stopWheels();

                startWheels(linePower);
                waitForTime(0.5);
                stopWheels();

                sense();
                climbers();

            }else{

                climbers();

            }


        }
    }

    /*

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

     */

    public void climbers() throws InterruptedException {

        leftPlow.setTargetPosition(armClimbers);
        leftPlow.setPower(autoArmPower);
        rightPlow.setTargetPosition(armClimbers);
        rightPlow.setPower(autoArmPower);

        waitForTime(1.25);

        leftPlow.setTargetPosition(0);
        leftPlow.setPower(-autoArmPower);
        rightPlow.setTargetPosition(0);
        rightPlow.setPower(-autoArmPower);


    }


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

        runWithoutEncoders();

        //distance = eyes.getUltrasonicLevel();

        //distance >= 20.0
        if(touchy.isPressed() == false) {

            telemetry.addData("Floor reading", String.format("%.4f %.4f", lightLeft, lightRight));
            telemetry.addData("Distance", distance);

            lightLeft = lightL.getLightDetected();
            lightRight = lightR.getLightDetected();
            //distance = eyes.getUltrasonicLevel();

            if(lightLeft < 0.1 || lightRight < 0.1){ //if its red, move forward a bit

                startWheels(linePower);
                waitForTime(lineTime);
                stopWheels();


            } else if(lightLeft > 0.8){

                startWheels(-linePower, linePower);
                waitForTime(lineTime);
                waitOneFullHardwareCycle();

            }else if(lightRight > 0.8){

                startWheels(linePower, -linePower);
                waitForTime(lineTime);
                stopWheels();
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

    public void waitForTime(double seconds) throws InterruptedException{
        startTime = getRuntime();
        while(getRuntime() < startTime + seconds){
            waitOneFullHardwareCycle();
        }
    }




}



