package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 11/13/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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

@Autonomous(name="TestAutoBeacon", group="Autonomous")
//@Disabled
public class TestAutoBeacon extends LinearOpMode{
     private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    GoldilocksHardware robot   = new GoldilocksHardware();   // Use a Pushbot's hardware
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

       moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 24, 24, 3.0);        //drive forward from wall to get closer to beacons
        sleep(150);
        telemetry.addLine("Move completed");

        moveThatRobot(GoldilocksHardware.TURN_SPEED, (1.5*Math.PI), -(1.5*Math.PI), 2.0);           //1.5 pi is 45 deg (CHECK DIRECTIONS)
        sleep(150);
        telemetry.addLine("Move completed");

        moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 63, 63, 6.0);         // drive towards beacons
        sleep(150);

        moveThatRobot(GoldilocksHardware.TURN_SPEED, 0, (3*Math.PI), 2.0);       // turn 45deg towards back beacon (1.5pi w/ left negative)
        sleep(150);
        telemetry.addLine("Move completed"); //cut it off here to test beacons!

        wallDistanceTest();

        //BEACON 1
        moveThatRobot(.2, 25, 25, 5.0);
        while ((robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) && (robot.whiteLineSensorOne.getLightDetected() < robot.lineLight)){
        }
        if (robot.whiteLineSensorOne.getLightDetected() > robot.lineLight){
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }

        /*moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 15, 15, 4.0);

        robot.leftMotor.setPower(.2);
        robot.rightMotor.setPower(.2);

        while (robot.whiteLineSensorOne.getLightDetected() < robot.lineLight){
        }


        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);*/
        bBeacon1();

        moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -15, -15, 4.0);

        wallDistanceTest();

        robot.leftMotor.setPower(-.2);
        robot.rightMotor.setPower(-.2);

        while (robot.whiteLineSensorOne.getLightDetected() < robot.lineLight){
        }


        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        bBeacon1();


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
            robot.buttonBopper.setPower(-.5);
        }
        while (!robot.touchBlue.isPressed()&& (robot.buttonBopper.getCurrentPosition()<robot.maxBop)&& opModeIsActive()){
        }

        robot.buttonBopper.setPower(0.);
        robot.buttonBopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!robot.touchBlue.isPressed()){
            stop();
            telemetry.addLine("stopped");
            telemetry.update();
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

            moveThatRobot(.2, -1.75, -1.75, 4.0);
            sleep(500);
            idle();
            if (robot.getBlueHue() < robot.midHue){
                moveThatRobot(.3, -3, -3, 2.0);
                moveThatBopper(robot.wallTouch + robot.beaconDepth);

                moveThatBopper(robot.wallTouch + robot.beaconDepth + robot.beaconClearance);
                //moveThatRobot(GoldilocksHardware.DRIVE_SPEED, -30, -30, 8.0);// distance to get close to the second beacon
            }
            else{ //if beacon is NOT blue then move to the next one, which is blue
                moveThatRobot(.3, -8.75, -8.75, 4.0);
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
            stop();
        }
    }

    public void distanceCorrect(){
        robot.driveCorrect = Math.abs(robot.wallTouch - 3500) / Math.sin(10);
    }


}





