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

    double multiplier = 0;

    /* Declare OpMode members. */
    GoldilocksHardware robot   = new GoldilocksHardware(this);   // Use Goldilocks' hardware

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.autoInit(hardwareMap);

        waitForStart();

        multiplier = isRed() ? -1. : 1.;

        //robot.runShooter(.45);

        double p = 0.5;
        int startPosition = robot.leftMotor.getCurrentPosition();
        robot.setLeftPower(p);
        robot.setRightPower(p);
        while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(20)){
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
        while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(41)){//42
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
                hitAngle = h >= 0;
            }
            else {
                hitAngle = h <= 0;
            }
        } while ( !hitAngle && opModeIsActive());
        robot.stopDriveMotors();
        checkOpModeActive();
        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();


        wallDistanceTest();
        checkOpModeActive();

        //BEACON 1
        float beaconHeading = robot.getHeading();
        //robot.moveThatRobot(.2, 25, 25, 5.0);
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

        robot.moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -15, -15, 4.0);
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

        //robot.moveThatRobot(DRIVE_SPEED, 48, 48, 4.0);  // S1: Forward 10 Inches with 30 Sec timeout
        //^from old code, moves robot forward to base

        //decrease shooter speed (disabled for testing)
       /* while ( p > 0.) {
            p -= .01;
            shooter.setPower(Math.abs(p));
            sleep(20);
        }
        shooter.setPower(0.);*/

    }


    //METHODS
    public void wallDistanceTest(){
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {
            robot.buttonBopper.setPower(isBlue() ? -.5 : .5);
        }
        if (isBlue()) {
            while (!robot.touchBlue.isPressed() && (robot.buttonBopper.getCurrentPosition() > -robot.maxBop) && opModeIsActive()) {}
        }
        else{
            while (!robot.touchRed.isPressed() && (robot.buttonBopper.getCurrentPosition() < robot.maxBop) && opModeIsActive()) {}
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

        moveThatBopper(robot.wallTouch + (int) (multiplier * (robot.beaconDepth + robot.beaconClearance)));

      /*  //FAR
        if (3500 > robot.wallTouch){
            robot.moveThatRobot(robot.TURN_SPEED, (.33 * Math.PI), -(.33 * Math.PI), 2.0);
            distanceCorrect();
            robot.moveThatRobot(robot.TURN_SPEED, -(.33 * Math.PI), (.33 * Math.PI), 2.0);
        }
        //CLOSE
        else if (robot.wallTouch > 3500){
            robot.moveThatRobot(robot.TURN_SPEED, -(.33 * Math.PI), (.33 * Math.PI), 2.0);
            distanceCorrect();
            robot.moveThatRobot(robot.TURN_SPEED, (.33 * Math.PI), -(.33 * Math.PI), 2.0);
        }*/
    }

    public void bBeacon1(){

        robot.moveThatRobot(.2, -2, -2, 1.5);
        checkOpModeActive();
        sleep(250);
        idle();


        if (isBlue() ? (robot.getBlueHue() < robot.midHue) : (robot.getBlueHue() > robot.midHue)) {
            robot.moveThatRobot(.3, -2.0, -2.0, 1.5);
            checkOpModeActive();
            moveThatBopper(robot.wallTouch + (int)(multiplier * robot.beaconDepth));
            moveThatBopper(robot.wallTouch + (int)(multiplier * (robot.beaconDepth + robot.beaconClearance)));
            //robot.moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -30, -30, 8.0);// distance to get close to the second beacon
        } else { //if beacon is NOT blue then move to the next one, which is blue
            robot.moveThatRobot(.3, -8.25, -8.25, 3.0);
            checkOpModeActive();
            moveThatBopper(robot.wallTouch + (int) (multiplier * robot.beaconDepth));
            moveThatBopper(robot.wallTouch + (int) (multiplier * (robot.beaconDepth + robot.beaconClearance)));
            //robot.moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 25, 25, 8.0);
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





