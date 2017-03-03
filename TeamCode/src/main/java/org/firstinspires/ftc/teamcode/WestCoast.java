/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@TeleOp(name="WestCoast OpMode", group="Linear Opmode")
public class WestCoast extends LinearOpMode {
    //connect hardware code with teleop code
    GoldilocksHardware robot           = new GoldilocksHardware(this);

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    double p=0.;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.teleInit(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

       // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //DRIVE CONTROL
            double lp = (double) gamepad1.left_stick_y;          //direct relationship: joystick-motor
            double rp = (double) gamepad1.right_stick_y;

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            Double tp = p;
            telemetry.addData("Shooter:", tp.toString());
            telemetry.addData("Bopper:", "enc: %d, line: %.3f",
                    robot.buttonBopper.getCurrentPosition(),
                    robot.whiteLineSensorOne.getLightDetected());
            telemetry.addData("Joysticks:", "left: %.2f, right: %.2f", lp, rp);
            telemetry.update();


            if (gamepad1.right_trigger > .5){                   //scale down drive speed for control
                lp = lp / 2;
                rp = rp / 2;
            }
            else if (gamepad1.y){
                lp = (-.2);
                rp = (-.2);
            }
            else if (gamepad1.a){
                lp = (.2);
                rp = (.2);
            }

            robot.leftMotor.setPower(-lp);               //set drive motor power
            robot.rightMotor.setPower(-rp);



            //SHOOTER
            double motorIncrement= .01;

            if (gamepad2.y){                    //12
                if (p < .7) {
                    p = p + motorIncrement;
                }
                else if (p >= .7) {
                    p = .7;
                }
            }
            else if (gamepad2.b){               //3
                if (p < .4) {
                    p = p + motorIncrement;
                }
                else if (p >= .4) {
                    p = .4;
                }
            }
            else if (gamepad2.a){               //6
                if (p < .5) {
                    p = p + motorIncrement;
                }
                else if (p >= .5) {
                    p = .5;
                }
            }
            else if (gamepad2.x){               //9
                if (p < .6) {
                    p = p + motorIncrement;
                }
                else if (p >= .6) {
                    p = .6;
                }
            }
            else{                               //slowly decrease power once button released
                if (p > 0.){
                    p = p - motorIncrement;
                    if (p < 0.){
                        p = 0.;
                    }
                }
                else{ //p <= 0 ?
                    p = 0.;
                }
            }

            robot.shooter.setPower(p);          //set shooter power


            //PARTICLE LIFT CONTROL
            if (gamepad2.dpad_up){
                robot.particleLift.setPosition(GoldilocksHardware.particleLiftUp);
            }
            else{
                robot.particleLift.setPosition(GoldilocksHardware.particleLiftDown);
            }


            //COLLECTOR CONTROL
            robot.collector.setPower(gamepad2.right_trigger-gamepad2.left_trigger);


            //COWCATCHER CONTROL
            //open both
            if (gamepad2.dpad_left || gamepad1.left_bumper){
                robot.ccLeft.setPosition(GoldilocksHardware.ccLeftOpen);
                robot.ccRight.setPosition(GoldilocksHardware.ccRightOpen);
            }
            //close both
            if (gamepad2.dpad_right || gamepad1.right_bumper){
                robot.ccRight.setPosition(GoldilocksHardware.ccRightClose);
                robot.ccLeft.setPosition(GoldilocksHardware.ccLeftClose);
            }

            //open right cc
            if (! GoldilocksHardware.rightOpen && gamepad2.right_bumper) {
                robot.ccRight.setPosition(GoldilocksHardware.ccRightOpen);
                sleep(100);
                GoldilocksHardware.rightOpen = true;
            }
            //close right cc
            else if (GoldilocksHardware.rightOpen && gamepad2.right_bumper){
                robot.ccRight.setPosition(GoldilocksHardware.ccRightClose);
                sleep(100);
                GoldilocksHardware.rightOpen = false;
            }

            //open left cc
            if (! GoldilocksHardware.leftOpen && gamepad2.left_bumper) {
                robot.ccLeft.setPosition(GoldilocksHardware.ccLeftOpen);
                sleep(100);
                GoldilocksHardware.leftOpen = true;
            }
            //close right cc
            else if (GoldilocksHardware.leftOpen && gamepad2.left_bumper){
                robot.ccLeft.setPosition(GoldilocksHardware.ccLeftClose);
                sleep(100);
                GoldilocksHardware.leftOpen = false;
            }

            //BUTTON BOPPER CONTROL
            if (gamepad1.x){
                robot.buttonBopper.setPower(-.5);

            }
            else if (gamepad1.b){
                robot.buttonBopper.setPower(.5);
            }
            else {
                robot.buttonBopper.setPower(0);
            }

            //stop them once they reach a certain encoder value (maxBop)
            // && (robot.buttonBopper.getCurrentPosition() > -(robot.maxBop))



            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
