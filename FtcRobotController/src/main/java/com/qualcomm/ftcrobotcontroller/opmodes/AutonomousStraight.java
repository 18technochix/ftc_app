package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Techno Team_PC_III on 3/8/2016.
 */
public class AutonomousStraight extends RobotOpMode {

    public void runOpMode() throws InterruptedException {

        auto = true;
        red = true;
        super.runOpMode(); //initalizes robot

        waitForStart();

        if (opModeIsActive()) {

            cowLeft.setPosition(cowLeftOpen);
            cowRight.setPosition(cowRightOpen);
            waitOneFullHardwareCycle();

            move(-90, 0.25);

            cowLeft.setPosition(cowLeftClosed);
            cowRight.setPosition(cowRightClosed);
            waitOneFullHardwareCycle();

            move(-10, 0.25);

        }
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

    }
}
