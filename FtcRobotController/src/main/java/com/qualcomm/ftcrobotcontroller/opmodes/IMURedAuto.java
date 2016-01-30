package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorController;


public class IMURedAuto extends RobotOpMode {
    double startTime;

    public void runOpMode() throws InterruptedException {

        auto = true;
        red = true;
        super.runOpMode(); //initalizes robot

        waitForStart();

        /*

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

                startWheels(-lopower);
                waitForTime(0.65);
                stopWheels();

                startWheels(lopower);
                waitForTime(0.73);
                stopWheels();

                sense();

            }

           // climbers();

        }
        */

    }


    /*

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

     */

    public void gyroSense(){

        /*
        * You could set it so that the values are stored in variavles up there
        * or that it returns an array of values?
        * hm.
        *
         */





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
                //push button
                return true;

            } else if (leftHue > hue && rightHue < hue) {
                //LEFT IS RED
               //push button
                return true;
            }



        }else{

            if (leftHue < hue && rightHue > hue) {
                // RIGHT IS RED
                //push button
                return true;
            } else if (leftHue > hue && rightHue < hue) {
                //LEFT IS RED
              //push button
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
                startWheels(lopower);
                //waitForTime(lineTime);
                turn(0.25, 0.4, true);
                waitOneFullHardwareCycle();

            } else if(lightRight > 0.75 && lightLeft < 0.3){

                turn(0.15, 0.4, true);
                startWheels(lopower);
                //waitForTime(lineTime);
                turn(0.25, 0.4, false);
                waitOneFullHardwareCycle();

            } else{
                startWheels(lopower);
                //waitForTime(lineTime);
                stopWheels();

            }

            return false;

        }else {

            return true;

        }

    }

    public void forward(double seconds, double power) throws InterruptedException {

        startWheels(power, power);

        waitForTime(seconds);

        stopWheels();

    }

    public void turn(double seconds, double power, boolean left) throws InterruptedException {

        if(left){

            startWheels(-power, power);

        }else {

            startWheels(power, -power);

        }

        waitForTime(seconds);

        stopWheels();

    }

    //Generic Methods //////////////////////////////////////////////////////////////////////////////

    public void waitForTime(double seconds) throws InterruptedException{
        startTime = getRuntime();
        while(getRuntime() < startTime + seconds){
            waitOneFullHardwareCycle();
        }
    } //end of wait for time void


}// end of auto class



