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

@TeleOp(name="WestCoast OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class WestCoast extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor shooter = null;
    DcMotor collector = null;
    DcMotor buttonBopper = null;
    Servo particleLift = null;
    Servo ccLeft = null;
    Servo ccRight = null;

       double p=0.;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        shooter = hardwareMap.dcMotor.get("shooter");
        buttonBopper = hardwareMap.dcMotor.get("button bopper");
        collector = hardwareMap.dcMotor.get("collector");
        ccLeft = hardwareMap.servo.get("cc left");
        ccRight = hardwareMap.servo.get("cc right");
        particleLift = hardwareMap.servo.get("particle lift");

        double ccLeftClose = (10./255.);
        double ccRightClose = (212./255.);
        double ccLeftOpen = (200./255.);
        double ccRightOpen = (50./255.);

        boolean leftOpen = false;
        boolean rightOpen = false;
        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        collector.setDirection(DcMotor.Direction.REVERSE);
        particleLift.setPosition(250./255.);
        ccRight.setPosition(ccRightClose);
        ccLeft.setPosition(ccLeftClose);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        leftMotor.setPower(0.);
        rightMotor.setPower(0.);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            Double tp = p;
            telemetry.addData("Shooter:", tp.toString());
            telemetry.update();

            double lp = gamepad1.left_stick_y;
            double rp = gamepad1.right_stick_y;

            //shooter
            double motorIncrement= .01;

            if (gamepad2.y){
                if (p < .8) {
                    p = p + motorIncrement;
                }
                else if (p >= .8) {
                    p = .8;
                                    }
                         }
            else if (gamepad2.b){
                if (p < .2) {
                    p = p + motorIncrement;
                }
                else if (p >= .2) {
                    p = .2;
                }
            }
            else if (gamepad2.a){
                if (p < .4) {
                    p = p + motorIncrement;
                }
                else if (p >= .4) {
                    p = .4;
                }
            }
            else if (gamepad2.x){
                if (p < .6) {
                    p = p + motorIncrement;
                }
                else if (p >= .6) {
                    p = .6;
                }
            }
            else{
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

//            gamepad1.left_trigger = a;
//
//            p + a = p;

            shooter.setPower(p);

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)

            if (gamepad1.right_trigger > .5){
                lp = lp / 2;
                rp = rp / 2;
            }

            leftMotor.setPower(lp);
            rightMotor.setPower(rp);

            //servo control
            if (gamepad2.dpad_up){
                particleLift.setPosition(190./255.);
            }
            else{
                particleLift.setPosition(250./255.);
            }

            //collector control

            collector.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            //cowcatcher control

            //open them
            if (gamepad2.dpad_left || gamepad1.dpad_left){
                ccLeft.setPosition(ccLeftOpen);
                sleep(100);
                ccRight.setPosition(ccRightOpen);
            }
            //close them
            if (gamepad2.dpad_right || gamepad1.dpad_right){
                ccRight.setPosition(ccRightClose);
                sleep(100);
                ccLeft.setPosition(ccLeftClose);
            }

            if (! rightOpen && (gamepad1.right_bumper || gamepad2.right_bumper)) {
                ccRight.setPosition(ccRightOpen);
                sleep(100);
                rightOpen = true;
            }
            else if (rightOpen && (gamepad1.right_bumper || gamepad2.right_bumper) ){
                ccRight.setPosition(ccRightClose);
                sleep(100);
                rightOpen = false;
            }

            if (! leftOpen && (gamepad1.left_bumper || gamepad2.left_bumper)) {
                ccLeft.setPosition(ccLeftOpen);
                sleep(100);
                leftOpen = true;
            }
            else if (leftOpen && (gamepad1.left_bumper || gamepad2.left_bumper) ){
                ccLeft.setPosition(ccLeftClose);
                sleep(100);
                leftOpen = false;
            }
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
