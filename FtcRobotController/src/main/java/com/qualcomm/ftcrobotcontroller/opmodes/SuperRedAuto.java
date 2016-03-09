package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Techno Team_PC_III on 3/8/2016.
 */
public class SuperRedAuto extends RobotOpMode {

    double startTime;

    double moveSpeed = 0.3;
    double sweepSpeed = 0.5;
    double turnSpeed = 0.7;

    double whiteValue = 0.9;

    boolean left = false;
    int turnCount = 0;
    final int finalTurnCount = 18;
    final int sweepDeg = 5;

    public void runOpMode() throws InterruptedException {

        auto = true;
        red = true;
        super.runOpMode(); //initalizes robot

        waitForStart();

        if (opModeIsActive()) {

            /*
            cowLeft.setPosition(cowLeftOpen);
            cowRight.setPosition(cowRightOpen);
            waitOneFullHardwareCycle();

            move(-14, moveSpeed);
            waitOneFullHardwareCycle();
            waitForTime(0.2);

            turn(34.0, turnSpeed);
            waitOneFullHardwareCycle();
            waitForTime(0.1);

            move(-77, moveSpeed);
            waitOneFullHardwareCycle();
            waitForTime(0.2);

            turn(40.0, turnSpeed);
            waitOneFullHardwareCycle();
            waitForTime(0.1);

            //get the debris out of the way

            cowLeft.setPosition(cowLeftOpen);
            cowRight.setPosition(cowRightOpen);
            waitOneFullHardwareCycle();
            waitForTime(0.5);

            cowLeft.setPosition(1);
            cowRight.setPosition(0);
            waitOneFullHardwareCycle();
            waitForTime(0.5);


            cowLeft.setPosition(0);
            cowRight.setPosition(1);
            waitOneFullHardwareCycle();
            */

            //make sure that the line is sensed and present

            while(findLine() == false){
                waitOneFullHardwareCycle();
            }

            //followLine();
            waitOneFullHardwareCycle();

            //sense();
            waitOneFullHardwareCycle();

            //climbers();

        }
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

            if(lightLeft > whiteValue){

                slightTurn(true, 0.3, 0.1);

            } else if(lightRight > whiteValue){

                slightTurn(false, 0.3, 0.1);

            } else{

                move(-2, 0.3);

            }

            waitOneFullHardwareCycle();


        }

    }

    public void slightTurn(boolean left, double power, double seconds) throws InterruptedException {

        runWheelsWithoutEncoders();

        if (!left) {
            fR.setPower(0);
            bR.setPower(0);
            fL.setPower(-power);
            bL.setPower(-power);
        }else{

            fR.setPower(-power);
            bR.setPower(-power);
            fL.setPower(0);
            bL.setPower(0);

        }

        waitForTime(seconds);

        stopWheels();

        waitOneFullHardwareCycle();

    }

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

        waitOneFullHardwareCycle();


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

        waitOneFullHardwareCycle();

    }//end of move void

    public boolean findLine() throws InterruptedException {
        beacon.setPosition(mid);
        waitOneFullHardwareCycle();

        while(touchy.isPressed() == false) {

             telemetry.addData("it's getting values lol", "okay? cool.");

             lightLeft = lightL.getLightDetected();
             lightRight = lightR.getLightDetected();

            waitOneFullHardwareCycle();

            telemetry.addData("Floor reading", String.format("%.4f %.4f", lightLeft, lightRight));

            waitOneFullHardwareCycle();

            if(lightLeft > whiteValue || lightRight > whiteValue) {
                telemetry.addData("one is sensed as white", "okay? cool.");
                return true;

            }else {

                if (!left && turnCount >= finalTurnCount) {
                    telemetry.addData("it turned right, switching left", "okay? cool.");
                    turnCount = 0;
                    turn(finalTurnCount * sweepDeg, turnSpeed);
                    left = true;

                } else if (left && turnCount >= finalTurnCount) {
                    telemetry.addData("it turned both ways, resetting program", "okay? cool.");
                    turn(-finalTurnCount * sweepDeg, turnSpeed);
                    move(-4, moveSpeed);
                    left = false;
                    return false;

                } else if (left) {
                    telemetry.addData("turning left", "okay? cool.");
                    turn(sweepDeg, sweepSpeed);
                    waitForTime(0.1);

                } else {
                    telemetry.addData("turning right", "okay? cool.");
                    turn(-sweepDeg, sweepSpeed);
                    waitForTime(0.1);

                }

                waitOneFullHardwareCycle();

            }

            waitOneFullHardwareCycle();
        }
        telemetry.addData("the button is presses LOL", "okay? cool.");
        return true;
    }

    public void waitForTime(double seconds) throws InterruptedException{
        startTime = getRuntime();
        while(getRuntime() < startTime + seconds){
            waitOneFullHardwareCycle();
        }
        waitOneFullHardwareCycle();
    } //end of wait for time void

    public void climbers() throws InterruptedException {

        dispL.setTargetPosition(climbersPosition);
        dispR.setTargetPosition(climbersPosition);
        waitOneFullHardwareCycle();
        dispL.setPower(dispPower);
        dispR.setPower(dispPower);
        waitOneFullHardwareCycle();

        while(dispL.getCurrentPosition() < climbersPosition){
            sleep(10);
            waitOneFullHardwareCycle();
        }

        dispL.setPower(0);
        dispR.setPower(0);

        climbers.setPosition(climbersOpen);
        waitOneFullHardwareCycle();

        waitForTime(0.75);
        waitOneFullHardwareCycle();

        dispL.setTargetPosition(0);
        dispR.setTargetPosition(0);
        dispL.setPower(-dispPower);
        dispR.setPower(-dispPower);
        waitOneFullHardwareCycle();

        while(dispL.getCurrentPosition() > 0){
            waitOneFullHardwareCycle();
        }

        dispL.setPower(0);
        dispR.setPower(0);
        waitOneFullHardwareCycle();

        climbers.setPosition(climbersClosed);
        waitOneFullHardwareCycle();



    }
}
