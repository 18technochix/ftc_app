package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 11/13/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Auto Base", group = "Autonomous")
@Disabled
class AutoBeaconBase extends LinearOpMode{
    // Constructor
    AutoBeaconBase(){}

    private ElapsedTime runtime = new ElapsedTime();

    private enum BeaconButton { BB_NEAR, BB_FAR, BB_NONE, BB_FALSE }

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

        robot.autoInit(hardwareMap, isBlue());

        while(!isStarted()){
            boolean even = (((int)(runtime.milliseconds()/250.) & 0x01) == 0);
            robot.setLED(isBlue(), even);
        }
        robot.setLED(isBlue(), true);

        //FIRST TURN
        multiplier = isBlue() ? -1 : 1;

        double p = 0.5;
        int startPosition = robot.leftMotor.getCurrentPosition();

        if (!opModeIsActive()){return;}

        boolean hitAngle = false;
        double h;

        moveThatBopperGo(-multiplier * robot.bopperRetract);

        swingToAngleEncoder(isBlue()? -45 : 45, 1.5);

        //SECOND TURN
        startPosition = robot.leftMotor.getCurrentPosition();
        robot.leftMotor.setPower(p);
        robot.rightMotor.setPower(p);

        int targetPosition = startPosition + robot.inchToEncoder(isBlue() ? 40.0 : 46.);
        while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< targetPosition) {
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
                hitAngle = h >= 0.;
            }
            else {
                hitAngle = h <= 0.;
            }
        } while ( !hitAngle && opModeIsActive());
        robot.stopDriveMotors();
        if (!opModeIsActive()){return;}
        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();

        pivotToAngleEncoder(isBlue() ? 0. : 2., .5);

        if (!opModeIsActive()){return;}

        //BEACON 1
        double creepySpeed = .25;
        double beaconHeading = robot.getHeading();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setLeftPower(creepySpeed);
        robot.setRightPower(creepySpeed);

        while ((robot.whiteLineSensor.getLightDetected() < robot.lineLight) && (opModeIsActive())) {double angle = robot.getHeading();
        }
        robot.stopDriveMotors();
        pivotToAngleEncoder(0, .5);
        if (!opModeIsActive()){return;}

        BeaconButton bb = hitBeacon();
        moveThatBopperGo(-multiplier * robot.bopperRetract);

        if (!opModeIsActive()){return;}
/*
        if (BeaconButton.BB_NONE == bb){
            telemetry.addData("correcting:", "first beacon not found");
            telemetry.update();
            findWall();
        }
        else if (BeaconButton.BB_FALSE == bb){
            telemetry.addData("continuing:", "COLOR SENSOR OFF");
            telemetry.update();
        }
*/

        //BEACON 2
        pivotToAngleEncoder(0, .5);
        robot.collector.setPower(-.5);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double distance = 36.; //53. +((bb == BeaconButton.BB_NEAR) ? 2. : 7.25);//48
        robot.moveThatRobot(.55*(isBlue() ? 1. : .95), .55*(isBlue() ? 1. : 1.),
                distance * (isBlue() ? 1. : .97), distance * (isBlue() ? 1. : 1.),
                6.0, "fast run");//speed was .65
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setLeftPower(creepySpeed);
        robot.setRightPower(creepySpeed);

        while ((robot.whiteLineSensor.getLightDetected() < robot.lineLight) && opModeIsActive()) {
            //ultrasonicDriveCorrect(robot.wallGap, -creepySpeed, .9);
        }

        robot.stopDriveMotors();
        if (!opModeIsActive()){return;}
        robot.shooter.setPower(robot.shooterPower = .2);
        pivotToAngleEncoder(0, .5);

        robot.shooter.setPower(robot.shooterPower = .4);
        bb = hitBeacon();
        moveThatBopperGo(0);

        if (!opModeIsActive()){return;}
        if (BeaconButton.BB_NONE == bb) {
            telemetry.addData("stopping:", "second beacon not found");
            telemetry.update();
            return;
        }

        robot.shooter.setPower(.8);
        if (!opModeIsActive() || BeaconButton.BB_NONE == bb){return;}

        //robot.moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -15, -15, 4.0);
        if (!opModeIsActive()){return;}
        //pivotToAngleEncoder(0.);
        //wallDistanceTest();

        turnShootDrive();

    }


    //METHODS

    BeaconButton hitBeacon(){
        BeaconButton bb;
        int bopperPush;
        double move = isBlue() ? 2 : 1.75;

        robot.moveThatRobot(.2, move, move, 1.5, "detect");//2.0
        checkOpModeActive();
        sleep(250);
        idle();
        double hue = robot.getHue(isBlue());
        telemetry.addData("hue:", hue);
        telemetry.update();
        if(robot.senseColor(hue) == GoldilocksHardware.ColorSensorResult.CS_UNKNOWN){
            sleep(2000);
            return BeaconButton.BB_FALSE;
        }

        if (isBlue() ? (robot.senseColor(hue) == GoldilocksHardware.ColorSensorResult.CS_BLUE) :
                (robot.senseColor(hue) == GoldilocksHardware.ColorSensorResult.CS_RED)) {
            bb = BeaconButton.BB_NEAR;
            move = isBlue() ? 3. : 2.75;
            robot.moveThatRobot(.3, move, move, 1.5, "bb_near");
            checkOpModeActive();
            if(robot.getDistance()>25.){return BeaconButton.BB_NONE;}

            robot.wallTouch = (int)((cmToIn(robot.getDistance()))*(double)robot.encoderPerInch);
            bopperPush = robot.wallTouch - robot.beaconDepth + robot.bopperSensorSpace + robot.bopperOvershoot - (robot.bopperWidth/2);
            bopperPush = Math.min(bopperPush,robot.maxBop);

            telemetry.addData("distance:", "%d", bopperPush);
            telemetry.update();

            moveThatBopperWait(multiplier * bopperPush, 1.5);
        }
        else { //if beacon is NOT blue then move to the next one, which is blue
            bb = BeaconButton.BB_FAR;
            move = isBlue() ? 8.5 : 8.;
            robot.moveThatRobot(.3, move, move, 1.5, "bb_far");
            checkOpModeActive();
            if(robot.getDistance()>25.){return BeaconButton.BB_NONE;}

            robot.wallTouch = (int)((cmToIn(robot.getDistance()))*(double)robot.encoderPerInch);
            bopperPush = robot.wallTouch - robot.beaconDepth + robot.bopperSensorSpace + robot.bopperOvershoot - (robot.bopperWidth/2);
            bopperPush = Math.min(bopperPush,robot.maxBop) ;

            telemetry.addData("distance:", "%d", robot.wallTouch);
            telemetry.update();

            moveThatBopperWait(multiplier * bopperPush, 1.5);
        }

        return bb;
    }

    public void moveThatBopperWait(int target, double timeout){
        moveThatBopperGo(target);

        runtime.reset();

        while (opModeIsActive() && robot.buttonBopper.isBusy() && (runtime.seconds() < timeout)){
        }
        robot.buttonBopper.setPower(0);
        if (!opModeIsActive()){
            return;
        }
    }

    void moveThatBopperGo(int target){
        robot.buttonBopper.setTargetPosition(target);
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.buttonBopper.setPower(.85);
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

    public void swingToAngleEncoder(double targetAngle, double timeout){
        sleep(200);
        double currentAngle = robot.getHeading();
        double deltaAngle = targetAngle-currentAngle;
        telemetry.addData("current heading:", currentAngle);
        telemetry.update();

        double distance = (24*Math.PI)*(deltaAngle/360.);

        if(distance > 0) {
            robot.moveThatRobot(.2, 0, distance, timeout, "angle");
        }
        else{
            robot.moveThatRobot(.2, -distance, 0, timeout, "angle"); //COULD THIS CAUSE AN ERROR IF DISTANCE = 0?
        }
        if (!opModeIsActive()){return;}
    }

    public void pivotToAngleEncoder(double targetAngle, double timeout){
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

    public void ultrasonicDriveCorrect(double targetDistance, double p, double multiplier) {
        ultrasonicDriveCorrect(targetDistance, p, p, multiplier);
    }

    public void ultrasonicDriveCorrect(double targetDistance, double lp, double rp, double multiplier){
        double lpr = lp * multiplier;
        double rpr = lp * multiplier;

        double distance = robot.getDistance();

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
        pivotToAngleEncoder(isBlue() ? -45: 45, 2.5);
        robot.collector.setPower(0);
        robot.doubleShot();
        robot.shooter.setPower(robot.shooterPower = .0);
        robot.moveThatRobot(.7, -65., -65., 5.0, "FINISH THE CAP BALLLLLL");//62
        robot.rampDownShooter();
    }

    public void findWall(){
        final int swing = 6;
        final double speed = .3;
        int startPosition = 0;
        double distance = robot.getDistance();

        while (distance > 30.){
            if (isBlue()) {
                startPosition = robot.leftMotor.getCurrentPosition();
                robot.setLeftPower(speed);
                robot.setRightPower(0);
                while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(swing)){}
                if (!opModeIsActive()){return;}
                startPosition = robot.rightMotor.getCurrentPosition();
                robot.setLeftPower(0);
                robot.setRightPower(speed);
                while (opModeIsActive() && robot.rightMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(swing)){}
                robot.stopDriveMotors();
                if (!opModeIsActive()){return;}
                distance = robot.getDistance();
            }
            else{
                startPosition = robot.rightMotor.getCurrentPosition();
                robot.setLeftPower(0);
                robot.setRightPower(speed);
                while (opModeIsActive() && robot.rightMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(swing)){}
                if (!opModeIsActive()){return;}
                startPosition = robot.leftMotor.getCurrentPosition();
                robot.setLeftPower(speed);
                robot.setRightPower(0);
                while (opModeIsActive() && robot.leftMotor.getCurrentPosition()< startPosition + robot.inchToEncoder(swing)){}
                robot.stopDriveMotors();
                if (!opModeIsActive()){return;}
                distance = robot.getDistance();
            }
            swingToAngleEncoder(0, 1.);
            if (!opModeIsActive()){return;}
        }

        return;
    }

}





