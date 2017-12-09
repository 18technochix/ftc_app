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
    Hardware robot = new Hardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.teleInit(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).






        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

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


            double angleDeg = 0; //robot.getHeading();
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

            if(gamepad1.right_bumper){
                fl = 0.2;
                fr = 0.2;
                bl = 0.2;
                br = 0.2;
            } else if(gamepad1.left_bumper){
                fl = -0.2;
                fr = -0.2;
                bl = -0.2;
                br = -0.2;
            } else if(gamepad1.left_trigger > 0.3){
                fl = -0.5;
                fr = 0.5;
                bl = 0.5;
                br = -0.5;
            } else if(gamepad1.right_trigger > 0.3){
                fl = 0.5;
                fr = -0.5;
                bl = -0.5;
                br = 0.5;
            }
            else {
                // Send calculated power to wheels
                fl = (s * Range.clip(fwd + rx + strafe, -1.0, 1.0)); //lyMod should just be fwd and lxMod should be strafe
                fr = (s * Range.clip(fwd + rx - strafe, -1.0, 1.0));
                bl = (s * Range.clip(fwd - rx - strafe, -1.0, 1.0));
                br = (s * Range.clip(fwd - rx + strafe, -1.0, 1.0));
            }

            robot.fl.setPower(fl);
            robot.fr.setPower(fr);
            robot.bl.setPower(bl);
            robot.br.setPower(br);

            if (gamepad2.left_bumper)
                robot.lift.setPower(.6);
            else if (gamepad2.right_bumper)
                robot.lift.setPower(-.6);



            if(gamepad1.dpad_left){
                if(robot.relicGrab.getPosition() != 0){
                    robot.relicGrab.setPosition(0);

                }else if(robot.relicGrab.getPosition() == 0){
                    robot.relicGrab.setPosition(180);
                }
            }

            if(gamepad1.dpad_up){
                robot.relicWrist.setPower(0.5);
            } else if (gamepad1.dpad_down){
                robot.relicWrist.setPower(-0.5);
            }

            if(gamepad2.dpad_up) {
                robot.relicElbow.setPower(0.5);
            } else if (gamepad2.dpad_up){
                robot.relicElbow.setPower(-0.5);
            }

            if(gamepad2.dpad_right) {
                robot.relicExtendArm1.setPower(0.5);
                robot.relicExtendArm2.setPower(0.5);
            } else if (gamepad2.dpad_left) {
                robot.relicExtendArm1.setPower(-0.5);
                robot.relicExtendArm2.setPower(-0.5);
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
}
