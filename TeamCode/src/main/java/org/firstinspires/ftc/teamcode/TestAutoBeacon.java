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

        //step through each leg of the path:
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //shooter auto control has been disabled for testing
        /*double p = 0;
        while (p < 0.45) {
            p += .01;
            shooter.setPower(p);
            sleep(20);
        }
        sleep(2500); //sleep for a second just to make sure the shooter is up to speed

        particleLift.setPosition(190. / 255.);
        sleep(1000);     // pause for servos to move
        particleLift.setPosition(250. / 255.);
        sleep(1500);     // pause for servos to move
        particleLift.setPosition(190. / 255.);
        sleep(500);     // pause for servos to move*/

        moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 39.75, 39.75, 4.0);        //drive forward from wall to get closer to beacons
        sleep(500);
        moveThatRobot(GoldilocksHardware.TURN_SPEED, 9.42, -9.42, 4.0);          //9.42 is roughly 3pi, which turns 90deg FOR BLUE
        sleep(500);
        moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 43.5, 43.5, 8.0);         // drive towards beacons
        sleep(500);
        moveThatRobot(GoldilocksHardware.TURN_SPEED, -28.27, 28.27, 4.0);       // turn 270deg to go backwards so that sensor is in front of pusher
        sleep(500);
                                                        //test wall distance
                                                        //move r&p back
                                                        //drive until white line (BACKWARDS DRIVE NOW)
                                                        //test r/b IF R continue IF B then...
                                                        //distanceWall- ~500
                                                        //move forward an inch or so to ensure button is pressed
                                                        //test wall distance
                                                        //move r&p back
                                                        //drive until white line (BACKWARDS DRIVE NOW)
                                                        //test r/b IF R continue IF B then...
                                                        //distanceWall- ~500
                                                        //move forward an inch or so to ensure button is pressed
                                                        //make it back to the base idk really

        moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 12, 12, 4.0);        //will be replaced with sensor
        sleep(500);
        moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 2, 2, 4.0);          //or another sensor
        sleep(500);
        moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 45, 45, 8.0);        //or drive until we reach the next white line
        sleep(500);
        moveThatRobot(GoldilocksHardware.DRIVE_SPEED, 2, 2, 4.0);          //or another sensor
        //go back to the cvbase

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
                }

                // Stop all motion;
                robot.leftMotor.setPower(0.);
                robot.rightMotor.setPower(0.);

                // Turn off RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }





        }
            /*else if (gamepad2.a){
                if (p < .2) {
                    p = p + motorIncrement;
                }
                else if (p >= .2) {
                    p = .2;
                }*/

        }





