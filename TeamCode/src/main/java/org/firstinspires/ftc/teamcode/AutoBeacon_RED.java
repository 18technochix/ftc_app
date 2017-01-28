package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by Techno Team_PC_III on 1/14/2017.
 */

@Autonomous (name="AutoBeacon RED", group="Autonomous")
@Disabled
public class AutoBeacon_RED extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    GoldilocksHardware robot   = new GoldilocksHardware(this);   // Use a Pushbot's hardware
    //private ElapsedTime     runtime = new ElapsedTime();

    public void runOpMode() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.autoInit(hardwareMap);

        waitForStart();
/*
        double p = 0;
        while (p < 0.45) {
            p += .01;
            robot.shooter.setPower(p);
            sleep(20);
        }
        sleep(2500); //sleep for a second just to make sure the shooter is up to speed

        robot.particleLift.setPosition(190. / 255.);
        sleep(1000);     // pause for servos to move
        robot.particleLift.setPosition(250. / 255.);
        sleep(1500);     // pause for servos to move
        robot.particleLift.setPosition(190. / 255.);
        sleep(500);     // pause for servos to move*/

        //moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 26, 26, 3.0);        //drive forward from wall to get closer to beacons
        //sleep(150);
        //telemetry.addLine("Move completed");

        double p = 0.5;
        int startPosition = robot.leftMotor.getCurrentPosition();
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(p);
        while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(26.5)){
        }
        if (!opModeIsActive()){return;}

        float targetAngle = 45;
        robot.rightMotor.setPower(p*.4);
        robot.leftMotor.setPower((p*.4)*.1);
        double h;
        do {
            h = robot.getHeading();
            telemetry.addData("heading: ", "%f", h);
            telemetry.update();
            sleep(20);
            idle();
        } while ( h <= targetAngle && opModeIsActive());
        if (!opModeIsActive()){return;}
  /*
        while (robot.getHeading() > targetAngle){
            sleep(20);
            idle();
        }*/

        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();

        startPosition = robot.leftMotor.getCurrentPosition();
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(p);
        while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(37)){
        }
        if (!opModeIsActive()){
            robot.stopDriveMotors();
            return;
        }


        targetAngle = 7; //CHECK TO SEE WHICH DIRECTION GYRO FAVORS, BLUE WAS -7
        robot.rightMotor.setPower((p*.4)*.1);
        robot.leftMotor.setPower(p*.4);
        do {
            h = robot.getHeading();
            telemetry.addData("heading: ", "%f", h);
            telemetry.update();
            sleep(20);
            idle();
        } while ( h >= targetAngle && opModeIsActive() );
        robot.stopDriveMotors();
        if (!opModeIsActive()){return;}
        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();


        wallDistanceTest();
        if (!opModeIsActive()){return;}

        //BEACON 1
        double beaconHeading = robot.getHeading();
        //moveThatRobot(.2, 25, 25, 5.0);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setPower(.2);
        robot.leftMotor.setPower(.2);

        //(robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) &&

        while ((robot.whiteLineSensorOne.getLightDetected() < robot.lineLight) && opModeIsActive()){
            telemetry.addData("linelight:", robot.whiteLineSensorOne.getLightDetected());
            telemetry.update();
        }

        robot.stopDriveMotors();
        if (!opModeIsActive()){return;}

        bBeacon1();
        if (!opModeIsActive()){return;}

        moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -15, -15, 4.0);
        if (!opModeIsActive()){return;}
        wallDistanceTest();
        if (!opModeIsActive()){return;}

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotor.setPower(-.2);
        robot.rightMotor.setPower(-.2);

        while ((robot.whiteLineSensorOne.getLightDetected() < robot.lineLight) && (opModeIsActive())){
        }
        robot.stopDriveMotors();
        if (!opModeIsActive()){return;}

        bBeacon1();
        if (!opModeIsActive()){return;}

        //moveThatRobot(DRIVE_SPEED, 48, 48, 4.0);  // S1: Forward 10 Inches with 30 Sec timeout
        //^from old code, moves robot forward to base

        //decrease shooter speed (disabled for testing)
       /* while ( p > 0.) {
            p -= .01;
            shooter.setPower(Math.abs(p));
            sleep(20);
        }
        shooter.setPower(0.);*/
    }




    public void moveThatRobot(double speed, double leftInches, double rightInches, double timeout){
        int newLeftTarget;
        int newRightTarget;
        int lCurrent;
        int rCurrent;
        int lPercent;
        int rPercent;

        //are we still running? good. if so:
        if (opModeIsActive()) {
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //now, where do we go? let's set the target position.
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * GoldilocksHardware.COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * GoldilocksHardware.COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            //now you gotta make sure they know what to do with this info. give the motor a runmode.
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.leftMotor.isBusy() || robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();

                    /*lCurrent = robot.leftMotor.getCurrentPosition();
                    rCurrent = robot.rightMotor.getCurrentPosition();

                    lPercent = lCurrent / newLeftTarget;
                    rPercent = rCurrent / newRightTarget;

                    //slowly ramp down the speed once 90% of target is reached
                    if ((lPercent > .75) || (rPercent > .75)){
                        speed = (speed/4);
                        if (speed < .1){
                            speed = .1;
                        }
                    }

                    // reset the timeout time and start motion.
                    runtime.reset();
                    robot.leftMotor.setPower(Math.abs(speed));
                    robot.rightMotor.setPower(Math.abs(speed));*/

                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0.);
            robot.rightMotor.setPower(0.);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }





    }

    //METHODS
    public void wallDistanceTest(){
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {
            robot.buttonBopper.setPower(.5);
        }
        while (!robot.touchRed.isPressed()&& (robot.buttonBopper.getCurrentPosition()<robot.maxBop)&& opModeIsActive()){
        }

        robot.buttonBopper.setPower(0.);
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!opModeIsActive()){return;}
        robot.wallTouch = robot.buttonBopper.getCurrentPosition();
        telemetry.addLine("wallTouch accounted for");
        telemetry.update();
        RobotLog.ii("technochix", "wall = %d", robot.wallTouch);

        moveThatBopper(robot.wallTouch - robot.beaconDepth - robot.beaconClearance);

      /*  //FAR
        if (3500 > robot.wallTouch){
            moveThatRobot(robot.TURN_SPEED, (.33 * Math.PI), -(.33 * Math.PI), 2.0);
            distanceCorrect();
            moveThatRobot(robot.TURN_SPEED, -(.33 * Math.PI), (.33 * Math.PI), 2.0);
        }
        //CLOSE
        else if (robot.wallTouch > 3500){
            moveThatRobot(robot.TURN_SPEED, -(.33 * Math.PI), (.33 * Math.PI), 2.0);
            distanceCorrect();
            moveThatRobot(robot.TURN_SPEED, (.33 * Math.PI), -(.33 * Math.PI), 2.0);
        }*/
    }

    public void bBeacon1(){

        moveThatRobot(.2, -2, -2, 1.5);
        if (!opModeIsActive()){return;}
        sleep(250);
        idle();
        if (robot.getBlueHue() > robot.midHue){
            moveThatRobot(.3, -2.75, -2.75, 1.5);
            if (!opModeIsActive()){return;}
            moveThatBopper(robot.wallTouch - robot.beaconDepth);

            moveThatBopper(robot.wallTouch - robot.beaconDepth - robot.beaconClearance);
            //moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -30, -30, 8.0);// distance to get close to the second beacon
        }
        else{ //if beacon is NOT red then move to the next one, which is red
            moveThatRobot(.3, -8.25, -8.25, 3.0);
            if (!opModeIsActive()){return;}
            moveThatBopper(robot.wallTouch - robot.beaconDepth);
            moveThatBopper(robot.wallTouch - robot.beaconDepth - robot.beaconClearance);
            //moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 25, 25, 8.0);
        }


    }

    public void moveThatBopper(int target){
        robot.buttonBopper.setTargetPosition(target);
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.buttonBopper.setPower(.5); //value is absolute, opmode will negate it if it needs to go backwards eg blue auto

        while (opModeIsActive() && robot.buttonBopper.isBusy()){
        }
        robot.buttonBopper.setPower(0);
        if (!opModeIsActive()){
            return;
        }
    }

    public void distanceCorrect(){
        robot.driveCorrect = Math.abs(robot.wallTouch - 3500) / Math.sin(10);
    }


}



