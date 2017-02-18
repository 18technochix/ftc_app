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
import org.firstinspires.ftc.robotcore.external.navigation.Position;

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

    private ElapsedTime runtime = new ElapsedTime();

    public enum BeaconButton { BB_NEAR, BB_FAR, BB_NONE }

    protected boolean red = true;

    public boolean isRed(){
        return red == true;
    }

    public boolean isBlue(){
        return red == false;
    }

    int multiplier = 0;

    /* Declare OpMode members. */
    GoldilocksHardware robot   = new GoldilocksHardware(this);   // Use Goldilocks' hardware

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.autoInit(hardwareMap);

        waitForStart();

        multiplier = isBlue() ? -1 : 1;

        double p = 0.5;
        int startPosition = robot.leftMotor.getCurrentPosition();
        robot.setLeftPower(p);
        robot.setRightPower(p);
        while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(22)){
            //driveAngleCompensation(0, p);
        }
        if (!opModeIsActive()){return;}

        boolean hitAngle = false;
        robot.setLeftPower((p*.4)* (isRed() ? .1 : 1.0));
        robot.setRightPower((p*.4)* (isBlue() ? .1 : 1.0));
        double h;
        do {
            h = robot.getHeading();
            telemetry.addData("heading: ", "%f", h);
            telemetry.update();
            sleep(20);
            idle();
            if (isBlue()){
                hitAngle = h <= -43;
            }
            else {
                hitAngle = h >= 45;
            }
        } while ( !hitAngle && opModeIsActive());
        if (!opModeIsActive()){return;}

        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();

        startPosition = robot.leftMotor.getCurrentPosition();
        robot.setLeftPower(p);
        robot.setRightPower(p);

        while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(isBlue() ? 40.5 : 42.75)) {
            // driveAngleCompensation(isBlue() ? -45 : 45, p);
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
        if (!opModeIsActive()){return;}
        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();

        turnToAngleEncoder(isBlue() ? 0. : 2., .5);
//        position = robot.getPosition();
//        telemetry.addData("Ending position:", "(%.3f, %.3f)", position.x, position.y);
//        telemetry.update();
        //sleep(10000);

        //wallDistanceTest();
        findWall();
        if (!opModeIsActive()){return;}

        //BEACON 1
        double creepySpeed = .25;
        double beaconHeading = robot.getHeading();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setLeftPower(-creepySpeed);
        robot.setRightPower(-creepySpeed);

        while ((robot.whiteLineSensorOne.getLightDetected() < robot.lineLight) && (opModeIsActive())) {double angle = robot.getHeading();
            //driveAngleCompensation(0, -creepySpeed);
            ultrasonicDriveCorrect(robot.wallGap, -creepySpeed, .9);
        }
        robot.stopDriveMotors();
        turnToAngleEncoder(0, .5);
        if (!opModeIsActive()){return;}

        BeaconButton bb = bBeacon1();
        if (!opModeIsActive() || BeaconButton.BB_NONE == bb){return;}

        //BEACON 2
        turnToAngleEncoder(0, .5);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double distance = 48 + 1.5 +((bb == BeaconButton.BB_NEAR) ? 2. : 7.25);
        robot.moveThatRobot(.55*(isBlue() ? 1. : 1.), .55*(isBlue() ? 1. : 1.), distance, distance, 6.0, "fast run");//speed was .65
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setLeftPower(-creepySpeed);
        robot.setRightPower(-creepySpeed);

        //(robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) &&

        while ((robot.whiteLineSensorOne.getLightDetected() < robot.lineLight) && opModeIsActive()) {
            //driveAngleCompensation(0, creepySpeed);
            ultrasonicDriveCorrect(robot.wallGap, creepySpeed, .9);
        }

        robot.stopDriveMotors();
        if (!opModeIsActive()){return;}
        robot.shooter.setPower(robot.shooterPower = .2);
        turnToAngleEncoder(0, .5);

        robot.shooter.setPower(robot.shooterPower = .4);
        bb = bBeacon1();
        robot.shooter.setPower(.55);
        if (!opModeIsActive() || BeaconButton.BB_NONE == bb){return;}

        //robot.moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -15, -15, 4.0);
        if (!opModeIsActive()){return;}
        //turnToAngleEncoder(0.);
        //wallDistanceTest();

        turnShootDrive();

    }


    //METHODS
    public boolean wallDistanceTest(){
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

        boolean foundWall = (robot.touchBlue.isPressed() || robot.touchRed.isPressed());

        robot.wallTouch = robot.buttonBopper.getCurrentPosition();

        if (!opModeIsActive()){
            return false;
        }

        int bopperMove = robot.wallTouch + (int) (multiplier * (robot.beaconDepth + robot.beaconClearance));
        telemetry.addData("bopper move:", "%d -> %d", robot.wallTouch, bopperMove);
        telemetry.update();
        moveThatBopper((foundWall ? bopperMove : 0), 1.5);
        return foundWall;
        // moveThatBopper(robot.wallTouch + (int) (multiplier * (robot.beaconDepth + robot.beaconClearance)));


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

    public BeaconButton bBeacon1(){
        BeaconButton bb;
        int bopperPush;
        int bopperRetract = 0;

        robot.moveThatRobot(.2, -1.5, -1.5, 1.5, "detect");//2.0
        checkOpModeActive();
        sleep(250);
        idle();

        if (isBlue() ? (robot.getBlueHue() < robot.midHue) : (robot.getBlueHue() > robot.midHue)) {
            bb = BeaconButton.BB_NEAR;
            robot.moveThatRobot(.3, -2.0, -2.0, 1.5, "bb_near");
            checkOpModeActive();
            if(robot.getDistance(isBlue())>25.){return BeaconButton.BB_NONE;}

            robot.wallTouch = (int)((cmToIn(robot.getDistance(isBlue())))*(double)robot.encoderConvert);
            bopperPush = multiplier*(robot.wallTouch - robot.beaconDepth);

            telemetry.addData("distance:", "%d", robot.wallTouch);
            telemetry.update();

            moveThatBopper(bopperPush, 1.5);
            moveThatBopper(bopperRetract, 1.5);
            //robot.moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -30, -30, 8.0);// distance to get close to the second beacon
        } else { //if beacon is NOT blue then move to the next one, which is blue
            double backup = isBlue() ? -7.25 : -8.25;
            bb = BeaconButton.BB_FAR;
            robot.moveThatRobot(.3, backup, backup, 1.5, "bb_far"); //8.25 & 3.0
            checkOpModeActive();
            if(robot.getDistance(isBlue())>25.){return BeaconButton.BB_NONE;}

            robot.wallTouch = (int)((cmToIn(robot.getDistance(isBlue())))*(double)robot.encoderConvert);
            bopperPush = multiplier*(robot.wallTouch - robot.beaconDepth);

            telemetry.addData("distance:", "%d", robot.wallTouch);
            telemetry.update();

            moveThatBopper(bopperPush, 1.5);
            moveThatBopper(bopperRetract, 1.5);
            //robot.moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 25, 25, 8.0);
        }

        return bb;
    }

    public void moveThatBopper(int target, double timeout){
        robot.buttonBopper.setTargetPosition(target);
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.buttonBopper.setPower(.7);

        runtime.reset();

        while (opModeIsActive() && robot.buttonBopper.isBusy() && (runtime.seconds() < timeout)){
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

    public void turnToAngleIMU(double targetAngle){
        double currentAngle = robot.getHeading();
        boolean turnRight = currentAngle-targetAngle > 0;

        robot.setLeftPower((.2)*(turnRight ? 1.0 : -1.0));
        robot.setRightPower((.2)*(turnRight ? -1.0 : 1.0));
        boolean hitAngle = false;
        do {
            currentAngle = robot.getHeading();
            telemetry.addData("current heading: ", "%f", currentAngle);
            telemetry.update();
            sleep(100); //20
            idle();
            if (turnRight){
                hitAngle = currentAngle <= targetAngle;
            }
            else {
                hitAngle = currentAngle >= targetAngle;
            }
        } while ( !hitAngle && opModeIsActive());
        robot.stopDriveMotors();
        if (!opModeIsActive()){return;}
        sleep(5000);
        telemetry.addData("final heading:", robot.getHeading());
        telemetry.update();
        sleep(15000);
    }

    public void turnToAngleEncoder(double targetAngle, double timeout){
        sleep(200);
        double currentAngle = robot.getHeading();
        double deltaAngle = targetAngle-currentAngle;
        telemetry.addData("current heading:", currentAngle);
        telemetry.update();

        double distance = (12*Math.PI)*(deltaAngle/360.);

        robot.moveThatRobot(.2, -distance, distance, timeout, "angle");//1.5

        if (!opModeIsActive()){return;}
       /* sleep(500);
        telemetry.addData("final heading:", robot.getHeading());
        telemetry.update();*/
        //sleep(15000);
    }

    public void driveAngleCompensation(double targetAngle, double power){
        double currentAngle = robot.getHeading();
        double deltaAngle = currentAngle-targetAngle;

        telemetry.addData("HEADING: ", "cur: %f, delt: %f", currentAngle, deltaAngle );
        telemetry.update();

        robot.setLeftPower((power)* ((deltaAngle < 0)^(power < 0) ? .7 : 1.0)); //.9
        robot.setRightPower((power)*((deltaAngle < 0)^(power < 0) ? 1.0 : .8));
    }

    public boolean findWall(){
        robot.wallTouch = isBlue() ? -2500 : 2500;
        return true;
    }

    public boolean findWallOld(){
        final int swing = 6;
        final double speed = .3;
        int startPosition = 0;
        while (!wallDistanceTest()){
            if (isBlue()) {
                startPosition = robot.leftMotor.getCurrentPosition();
                robot.setLeftPower(speed);
                robot.setRightPower(0);
                while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(swing)){}
                if (!opModeIsActive()){return false;}
                startPosition = robot.rightMotor.getCurrentPosition();
                robot.setLeftPower(0);
                robot.setRightPower(speed);
                while (opModeIsActive() && robot.rightMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(swing)){}
                robot.stopDriveMotors();
                if (!opModeIsActive()){return false;}
            }
            else{
                startPosition = robot.rightMotor.getCurrentPosition();
                robot.setLeftPower(0);
                robot.setRightPower(speed);
                while (opModeIsActive() && robot.rightMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(swing)){}
                if (!opModeIsActive()){return false;}
                startPosition = robot.leftMotor.getCurrentPosition();
                robot.setLeftPower(speed);
                robot.setRightPower(0);
                while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(swing)){}
                robot.stopDriveMotors();
                if (!opModeIsActive()){return false;}
            }
            turnToAngleEncoder(0, .5);
            if (!opModeIsActive()){return false;}
        }

        return true;
    }

    public void ultrasonicDriveCorrect(double targetDistance, double p, double multiplier) {
        ultrasonicDriveCorrect(targetDistance, p, p, multiplier);
    }

    public void ultrasonicDriveCorrect(double targetDistance, double lp, double rp, double multiplier){
        double lpr = lp * multiplier;
        double rpr = lp * multiplier;

        double distance = robot.getDistance(isBlue());

        if(distance > targetDistance){
            robot.setLeftPower(isBlue() ? lp : lpr);
            robot.setRightPower(isBlue() ? rpr : rp);
        }
        else if(distance < targetDistance){
            robot.setLeftPower(isBlue() ? lpr : lp);
            robot.setRightPower(isBlue() ? rp : rpr);
        }
        else{
            robot.setLeftPower(lp);
            robot.setRightPower(rp);
        }

    }

    public double cmToIn(double cm){
        return cm * .39;
    }

    public void turnShootDrive(){
        turnToAngleEncoder(isBlue() ? 135: -135, 2.5);
        robot.doubleShot();
        robot.shooter.setPower(robot.shooterPower = .0);
        robot.moveThatRobot(.5, 62., 62., 5.0, "FINISH THE CAP BALLLLLL");
        robot.rampDownShooter();
    }

}





