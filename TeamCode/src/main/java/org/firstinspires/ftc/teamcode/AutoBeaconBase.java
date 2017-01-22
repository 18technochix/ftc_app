package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 11/13/2016.
 */

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Base", group="Autonomous")
@Disabled
public class AutoBeaconBase extends LinearOpMode{
    // Constructor
    public AutoBeaconBase(){}

    protected boolean red = true;

    public boolean isRed(){
        return red == true;
    }

    public boolean isBlue(){
        return red == false;
    }

    /* Declare OpMode members. */
    GoldilocksHardware robot   = new GoldilocksHardware(this);   // Use Goldilocks' hardware

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.autoInit(hardwareMap);

        waitForStart();

        //robot.runShooter(.45);

        double p = 0.5;
        int startPosition = robot.leftMotor.getCurrentPosition();
        robot.setLeftPower(p);
        robot.setRightPower(p);
        while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(26.5)){
        }
        checkOpModeActive();

        boolean hitAngle = false;
        robot.setLeftPower((p*.4)* (isRed() ? .1 : 1.0));
        robot.setRightPower((p*.4)* (isBlue() ? .1 : 1.0));
        float h;
        do {
            h = robot.getHeading();
            telemetry.addData("heading: ", "%f", h);
            telemetry.update();
            sleep(20);
            idle();
            if (isBlue()){
                hitAngle = h <= -45;
            }
            else {
                hitAngle = h >= 45;
            }
        } while ( !hitAngle && opModeIsActive());
        checkOpModeActive();

        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();

        startPosition = robot.leftMotor.getCurrentPosition();
        robot.setLeftPower(p);
        robot.setRightPower(p);
        while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(42)){
        }
        if (!opModeIsActive()){
            robot.stopDriveMotors();
            return;
        }



        robot.setLeftPower((p*.4)*(isBlue() ? .1 : 1.0));
        robot.setRightPower((p*.4)*(isRed() ? .1 : 1.0));
        hitAngle = false;
        do {
            h = robot.getHeading();
            telemetry.addData("heading: ", "%f", h);
            telemetry.update();
            sleep(20);
            idle();
            if (isBlue()){
                hitAngle = h >= 7;
            }
            else {
                hitAngle = h <= 0;
            }
        } while ( !hitAngle && opModeIsActive());
        robot.stopDriveMotors();
        checkOpModeActive();
        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();

        if (true){
            return;}
        else {
            wallDistanceTest();
            checkOpModeActive();

            //BEACON 1
            float beaconHeading = robot.getHeading();
            //moveThatRobot(.2, 25, 25, 5.0);
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setLeftPower(.2);
            robot.setRightPower(.2);

            //(robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) &&

            while ((robot.whiteLineSensorOne.getLightDetected() < robot.lineLight) && opModeIsActive()) {
            }

            robot.stopDriveMotors();
            checkOpModeActive();

            bBeacon1();
            checkOpModeActive();

            moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -15, -15, 4.0);
            checkOpModeActive();
            wallDistanceTest();
            checkOpModeActive();


            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.setLeftPower(-.2);
            robot.setRightPower(-.2);

            while ((robot.whiteLineSensorOne.getLightDetected() < robot.lineLight) && (opModeIsActive())) {
            }
            robot.stopDriveMotors();
            checkOpModeActive();

            bBeacon1();
            checkOpModeActive();

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
                robot.runtime.reset();
                robot.setLeftPower(Math.abs(speed));
                robot.setRightPower(Math.abs(speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opModeIsActive() &&
                        (robot.runtime.seconds() < timeout) &&
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
                    robot.setLeftPower(Math.abs(speed));
                    robot.setRightPower(Math.abs(speed));*/

                    idle();
                }

                // Stop all motion;
                robot.setLeftPower(0.);
                robot.setRightPower(0.);

                // Turn off RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
    }

    //METHODS
    public void wallDistanceTest(){
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {
            robot.buttonBopper.setPower(-.5);
        }
        while (!robot.touchBlue.isPressed()&& (robot.buttonBopper.getCurrentPosition()>-3700)&& opModeIsActive()){
        }

        robot.buttonBopper.setPower(0.);
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!opModeIsActive()){
            return;
        }
        robot.wallTouch = robot.buttonBopper.getCurrentPosition();
        telemetry.addLine("variable accounted for");
        telemetry.update();
        RobotLog.ii("technochix", "wall = %d", robot.wallTouch);

        moveThatBopper(robot.wallTouch + robot.beaconDepth + robot.beaconClearance);

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
            checkOpModeActive();
            sleep(250);
            idle();
            if (robot.getBlueHue() < robot.midHue){
                moveThatRobot(.3, -2.0, -2.0, 1.5);
                checkOpModeActive();
                moveThatBopper(robot.wallTouch + robot.beaconDepth);
                moveThatBopper(robot.wallTouch + robot.beaconDepth + robot.beaconClearance);
                //moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -30, -30, 8.0);// distance to get close to the second beacon
            }
            else{ //if beacon is NOT blue then move to the next one, which is blue
                moveThatRobot(.3, -8.25, -8.25, 3.0);
                checkOpModeActive();
                moveThatBopper(robot.wallTouch + robot.beaconDepth);
                moveThatBopper(robot.wallTouch + robot.beaconDepth + robot.beaconClearance);
                //moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 25, 25, 8.0);
            }


    }

    public void moveThatBopper(int target){
        robot.buttonBopper.setTargetPosition(target);
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.buttonBopper.setPower(.5);

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

    public void checkOpModeActive(){
        if (!opModeIsActive()){return;}
    }



}





