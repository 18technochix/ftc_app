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
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor shooter = null;
    Servo particleLift = null;
    Servo ccLeft = null;
    Servo ccRight = null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    //HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    //private ElapsedTime     runtime = new ElapsedTime();

    static final double     ENCODER_CPR             = 1120 ;    // AndyMark encoder count
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (ENCODER_CPR * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4; //0.6;
    static final double     SHOOTER_SPEED           = 0.2;
    static final double     TURN_SPEED              = 0.1;

    static final double CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER_INCHES;
    //static final double ROTATIONS =

    public void runOpMode() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        shooter = hardwareMap.dcMotor.get("shooter");
        particleLift = hardwareMap.servo.get("particle lift");
        ccLeft = hardwareMap.servo.get("cc left");
        ccRight = hardwareMap.servo.get("cc right");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //reset encoders

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        double ccLeftClose = (10./255.);
        double ccRightClose = (212./255.);

        leftMotor.setPower(0.);
        rightMotor.setPower(0.);
        shooter.setPower(0.);
        particleLift.setPosition(250. / 255.);
        ccRight.setPosition(ccRightClose);
        ccLeft.setPosition(ccLeftClose);

        waitForStart();

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

        moveThatRobot(DRIVE_SPEED, 39.75, 39.75, 4.0);        //drive forward from wall to get closer to beacons
        sleep(500);
        moveThatRobot(TURN_SPEED, 9.42, -9.42, 4.0);          //9.42 is roughly 3pi, which turns 90deg FOR BLUE
        sleep(500);
        moveThatRobot(DRIVE_SPEED, 43.5, 43.5, 8.0);         // drive towards beacons
        sleep(500);
        moveThatRobot(TURN_SPEED, -28.27, 28.27, 4.0);  // turn 270deg to go backwards so that sensor is in front of pusher
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

        moveThatRobot(DRIVE_SPEED, 12, 12, 4.0);        //will be replaced with sensor
        sleep(500);
        moveThatRobot(DRIVE_SPEED, 2, 2, 4.0);          //or another sensor
        sleep(500);
        moveThatRobot(DRIVE_SPEED, 45, 45, 8.0);        //or drive until we reach the next white line
        sleep(500);
        moveThatRobot(DRIVE_SPEED, 2, 2, 4.0);          //or another sensor
        //go back to the cvbase

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //moveThatRobot(DRIVE_SPEED, 48, 48, 4.0);  // S1: Forward 10 Inches with 30 Sec timeout
        //^from old code, moves robot forward to base
        //moveThatRobot(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //moveThatRobot(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout



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
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //now, where do we go? let's set the target position.
                newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                leftMotor.setTargetPosition(newLeftTarget);
                rightMotor.setTargetPosition(newRightTarget);

                //now you gotta make sure they know what to do with this info. give the motor a runmode.
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftMotor.setPower(Math.abs(speed));
                rightMotor.setPower(Math.abs(speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeout) &&
                        (leftMotor.isBusy() || rightMotor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Path2",  "Running at %7d :%7d",
                            leftMotor.getCurrentPosition(),
                            rightMotor.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                leftMotor.setPower(0.);
                rightMotor.setPower(0.);

                // Turn off RUN_TO_POSITION
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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





