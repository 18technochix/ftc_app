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

            move(-14, 0.5);
            waitOneFullHardwareCycle();
            waitForTime(0.2);
            turn(34.0, 0.4);
            waitOneFullHardwareCycle();
            waitForTime(0.1);
            move(-80, 0.5);
            waitOneFullHardwareCycle();
            waitForTime(0.2);
            turn(34.0, 0.4);

            // start line sensing
            followLine();
            sense();

            /*

            forward off the wall
            turn cc to be parallel with the line
            forward until you reach the red box
            turn cc to be perpendicular to the red box
            maybe forward a bit
            follow line forward until you hit the beacon with the touch sensor
            program that senses the colors by moving the servo, then picking a side
            then whamming into and back out of the beacon
            then backing up a bit more
            how do we get out of here?

             */

        }


    }


    /*

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

     */


    public void turn(double angle, double power) throws InterruptedException {

        runWheelsWithoutEncoders();

        double startingAngle;
        double targetAngle;

        getAngles();

        startingAngle = yawAngle[0];

        printYawData();


        if(angle < 0){

            if(startingAngle + angle < -180.0){

                //telemetry?

                targetAngle = (360 + (startingAngle + angle));

                turnRight(power);

                while ((yawAngle[0] < 0 && yawAngle[0] > -180) || (yawAngle [0] > 0 && yawAngle[0] > targetAngle)) {

                    refreshIMU();
                    waitOneFullHardwareCycle();

                }


            }else{

                targetAngle = startingAngle + angle;

                turnRight(power);

                while (yawAngle[0] > targetAngle) {

                    refreshIMU();
                    waitOneFullHardwareCycle();

                }


            }

        }else if(angle > 0){

            if(startingAngle + angle > 180){

                targetAngle = (startingAngle + angle) - 360;

                turnLeft(power);

                while ((yawAngle[0] > 0 && yawAngle[0] < 180) || (yawAngle [0] < 0 && yawAngle[0] < targetAngle)) {

                    refreshIMU();
                    waitOneFullHardwareCycle();

                }

            }else{

                targetAngle = startingAngle + angle;

                turnLeft(power);

                while(yawAngle[0] < targetAngle){

                    refreshIMU();
                    waitOneFullHardwareCycle();

                }

            }


        }

        stopWheels();


    }


    public boolean sense() throws InterruptedException {

        //back up a tiny bit
        move(4, 0.3);
        waitOneFullHardwareCycle();

        float hsvValues [] = {0F,0F,0F};

        moveBeacon(midLeft);
        waitOneFullHardwareCycle();

        waitForTime(0.5);

        int leftBlue = fruity.blue();
        int leftRed = fruity.red();
        int leftGreen = fruity.green();
        Color.RGBToHSV((leftRed * 255) / 800, (leftGreen * 255) / 800, (leftBlue * 255) / 800, hsvValues);
        float leftHue = hsvValues[0];
        waitOneFullHardwareCycle();

        waitForTime(0.5);

        moveBeacon(midRight);
        waitOneFullHardwareCycle();

        waitForTime(2);

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

        if(red) {

            if (leftHue < hue && rightHue > hue) {
                moveBeacon(fullRight);
                waitOneFullHardwareCycle();
                move(-5, 0.7);
                waitOneFullHardwareCycle();
                move(5, 0.3);
                waitOneFullHardwareCycle();
                return true;

            } else if (leftHue > hue && rightHue < hue) {
                moveBeacon(fullLeft);
                waitOneFullHardwareCycle();
                move(-5, 0.7);
                waitOneFullHardwareCycle();
                move(5, 0.3);
                waitOneFullHardwareCycle();
                return true;
            }



        }else{

            if (leftHue < hue && rightHue > hue) {
                moveBeacon(fullLeft);
                waitOneFullHardwareCycle();
                move(-5, 0.7);
                waitOneFullHardwareCycle();
                move(5, 0.3);
                waitOneFullHardwareCycle();
                return true;
            } else if (leftHue > hue && rightHue < hue) {
                moveBeacon(fullRight);
                waitOneFullHardwareCycle();
                move(-5, 0.7);
                waitOneFullHardwareCycle();
                move(5, 0.3);
                waitOneFullHardwareCycle();
                return true;
            }



        }


        return false;


    }

    public void followLine() throws InterruptedException {

        beacon.setPosition(mid); //open beacon sensor

        waitOneFullHardwareCycle();

        //close cows
        //cowLeft.setPosition(0);
       // cowRight.setPosition(1);

        while(touchy.isPressed() == false) {

            telemetry.addData("Floor reading", String.format("%.4f %.4f", lightLeft, lightRight));

            lightLeft = lightL.getLightDetected();
            lightRight = lightR.getLightDetected();

            if(lightLeft > 0.65 && lightRight < 0.33){

                slightTurn(true, 0.3, 0.1);

            } else if(lightRight > 0.75 && lightLeft < 0.3){

                slightTurn(false, 0.3, 0.1);

            } else{

                move(-2, 0.3);

            }

            waitOneFullHardwareCycle();


        }

    }


    public void move(double inches, double power) throws InterruptedException {

        resetWheelEncoders();
        runWheelsToPosition();

        fR.setTargetPosition(counts(inches));
        fL.setTargetPosition(counts(inches));
        bR.setTargetPosition(counts(inches));
        bL.setTargetPosition(counts(inches));

        waitOneFullHardwareCycle();

        startWheels(power);

        waitOneFullHardwareCycle();

        while(!atWheelPosition(counts(inches))){
            telemetry.addData("Pos:", String.format("%03d %03d %03d %03d", bR.getCurrentPosition(),
                    fR.getCurrentPosition(), bL.getCurrentPosition(), bR.getCurrentPosition()));
            waitOneFullHardwareCycle();
        }

        stopWheels();

    }

    public void slightTurn(boolean left, double power, double seconds) throws InterruptedException {

        runWheelsWithoutEncoders();

        if (!left) {
            fR.setPower(0);
            bR.setPower(0);
            fL.setPower(power);
            bL.setPower(power);
        }else{

            fR.setPower(power);
            bR.setPower(power);
            fL.setPower(0);
            bL.setPower(0);

        }

        waitForTime(seconds);

        stopWheels();

        waitOneFullHardwareCycle();

    }


    //Generic Methods //////////////////////////////////////////////////////////////////////////////

    public void waitForTime(double seconds) throws InterruptedException{
        startTime = getRuntime();
        while(getRuntime() < startTime + seconds){
            waitOneFullHardwareCycle();
        }
        waitOneFullHardwareCycle();
    } //end of wait for time void


}// end of auto class



