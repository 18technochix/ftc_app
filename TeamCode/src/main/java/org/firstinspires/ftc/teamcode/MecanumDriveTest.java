/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum OpMode Test", group="Linear Opmode")

public class MecanumDriveTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor lift = null;
    private Servo grabServo = null;

    BNO055IMU gyro;

    double GRAB_OPEN = 0.12;
    double GRAB_CLOSE = 0.63;
    double servoPosition = ((GRAB_CLOSE - GRAB_OPEN)/2) + GRAB_OPEN;




    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeft  = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight  = hardwareMap.get(DcMotor.class, "fr");
        lift = hardwareMap.get(DcMotor.class, "lift");
        grabServo = hardwareMap.get(Servo.class, "grabServo");
        gyro = hardwareMap.get(BNO055IMU.class, "imu");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);

        Orientation angles;



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        grabServo.setDirection(Servo.Direction.FORWARD);
        grabServo.setPosition(servoPosition);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;

            double rx = gamepad1.right_stick_x;
            double ry = -gamepad1.right_stick_y;


             // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            /*
            fl = ly + rx + lx
            bl = ly + rx - lx
            fr = ly - rx - lx
            br = ly - rx + lx
             */

            double s = .5;


            double angleDeg = 0; //getHeading();
            double angleRad = angleDeg * (Math.PI / 180);
            double fwd = (ly * Math.cos(angleRad)) + (lx * Math.sin(angleRad));
            double strafe = (-lx * Math.sin(angleRad)) + (ly * Math.cos(angleRad));
            double lyMod = fwd * ly;
            double lxMod = strafe * lx;
            lyMod = ly;
            lxMod = lx;

            double fl = 0;
            double fr = 0;
            double bl = 0;
            double br = 0;

            if(gamepad1.dpad_up){
                fl = 0.2;
                fr = 0.2;
                bl = 0.2;
                br = 0.2;
            } else if(gamepad1.dpad_down){
                fl = -0.2;
                fr = -0.2;
                bl = -0.2;
                br = -0.2;
            } else if(gamepad1.dpad_left){
                fl = -0.5;
                fr = 0.5;
                bl = 0.5;
                br = -0.5;
            } else if(gamepad1.dpad_right){
                fl = 0.5;
                fr = -0.5;
                bl = -0.5;
                br = 0.5;
            }
            else {
                // Send calculated power to wheels
                fl = (s * Range.clip(lyMod + rx + lxMod, -1.0, 1.0)); //lyMod should just be fwd and lxMod should be strafe
                fr = (s * Range.clip(lyMod + rx - lxMod, -1.0, 1.0));
                bl = (s * Range.clip(lyMod - rx - lxMod, -1.0, 1.0));
                br = (s * Range.clip(lyMod - rx + lxMod, -1.0, 1.0));
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            if (gamepad2.left_bumper)
                lift.setPower(.6);
            else if (gamepad2.right_bumper)
                lift.setPower(-.6);
            else
                lift.setPower(0);

            if(gamepad2.dpad_right){
                servoPosition += .01;
                if(servoPosition > GRAB_CLOSE){
                    servoPosition = GRAB_CLOSE;
                }
                grabServo.setPosition(servoPosition);

            }else if(gamepad2.dpad_left){
                servoPosition -= .01;
                if(servoPosition < GRAB_OPEN){
                    servoPosition = GRAB_OPEN;
                }
                grabServo.setPosition(servoPosition);
            }










           /* double sPosition = (gamepad2.right_stick_x + 1.0)/2.0;
            Range.clip(sPosition, GRAB_OPEN, GRAB_CLOSE);
            grabServo.setPosition(sPosition);*/


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", motorPower, rightPower);
            telemetry.update();
        }

    }
    public double getHeading(){
        Orientation angles = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double angle = angles.firstAngle;
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angle));
    }

}
