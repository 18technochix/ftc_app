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


        if (opModeIsActive()) {

            /*
            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            telemetry.addData("Headings(yaw): ",
                    String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
            turnRight(0.7);
            waitOneFullHardwareCycle();
            */

            turn(90.0, 0.3);
            turn(100.0, 0.3);

        }


    }


    /*

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

     */


    public void turn(double angle, double power) throws InterruptedException {

        double startingAngle;
        double targetAngle;

        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

        startingAngle = yawAngle[0];

        telemetry.addData("Headings(yaw): ",
                String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));


        if(angle < 0){

            if(startingAngle + angle < -180.0){

                targetAngle = (360 + (startingAngle + angle));

                turnLeft(power);

                while ((yawAngle[0] < 0 && yawAngle[0] > -180) || (yawAngle [0] > 0 && yawAngle[0] > targetAngle)) {

                    gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
                    telemetry.addData("Headings(yaw): ",
                            String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
                    waitOneFullHardwareCycle();

                }


            }else{

                targetAngle = startingAngle + angle;

                turnLeft(power);

                while (yawAngle[0] < targetAngle) {

                    gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
                    telemetry.addData("Headings(yaw): ",
                            String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
                    waitOneFullHardwareCycle();

                }


            }

        }else if(angle > 0){

            if(startingAngle + angle > 180){

                targetAngle = (startingAngle + angle) - 360;

                turnRight(power);

                while ((yawAngle[0] > 0 && yawAngle[0] < 180) || (yawAngle [0] < 0 && yawAngle[0] < targetAngle)) {

                    gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
                    telemetry.addData("Headings(yaw): ",
                            String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
                    waitOneFullHardwareCycle();

                }

            }else{

                targetAngle = startingAngle + angle;

                turnRight(power);

                while(yawAngle[0] > targetAngle){

                    gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
                    telemetry.addData("Headings(yaw): ",
                            String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
                    waitOneFullHardwareCycle();

                }

            }


        }

        stopWheels();


    }


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

        fR.setPower(0);
        bR.setPower(0);
        fL.setPower(power);
        bL.setPower(power);

    }

    public void turnRight(double power){

        fR.setPower(power);
        bR.setPower(power);
        fL.setPower(0);
        bL.setPower(0);

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

                //turn(0.15, 0.4, false);
                startWheels(lopower);
                //waitForTime(lineTime);
                //turn(0.25, 0.4, true);
                waitOneFullHardwareCycle();

            } else if(lightRight > 0.75 && lightLeft < 0.3){

                // turn(0.15, 0.4, true);
                startWheels(lopower);
                //waitForTime(lineTime);
                // turn(0.25, 0.4, false);
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

        startWheels(power);

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



