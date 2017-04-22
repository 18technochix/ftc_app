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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LED;
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

@TeleOp(name="LED Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class LightTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    DeviceInterfaceModule dim;                  // Device Object
    //DigitalChannel        dcLedOut;               // Device Object
    LED                   led1;
    LED                   led2;
    LED                   led3;
    LED                   led4;
    LED                   led5;
    LED                   led6;
    LED                   led7;

    AnalogInput           photoR;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");   //  Use generic form of device mapping
        led1 = hardwareMap.get(LED.class, "led 1");
        led2 = hardwareMap.get(LED.class, "led 2");
        led3 = hardwareMap.get(LED.class, "led 3");
        led4 = hardwareMap.get(LED.class, "led 4");
        led5 = hardwareMap.get(LED.class, "led 5");
        led6 = hardwareMap.get(LED.class, "led 6");
        led7 = hardwareMap.get(LED.class, "led 7");

        photoR = hardwareMap.get(AnalogInput.class, "photo resistor");

        //dcLedOut.setMode(DigitalChannelController.Mode.OUTPUT);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean b1 = true;
        boolean b2 = false;
        boolean b3 = true;
        boolean b4 = false;
        boolean b5 = true;
        boolean b6 = false;
        boolean b7 = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("voltage: ", "%.3f", photoR.getVoltage() );
            telemetry.update();

            led1.enable(b1);
            led2.enable(b2);
            led3.enable(b3);
            led4.enable(b4);
            led5.enable(b5);
            led6.enable(b6);
            led7.enable(b7);

            b1 = !b1;
            b2 = !b2;
            b3 = !b3;
            b4 = !b4;
            b5 = !b5;
            b6 = !b6;
            b7 = !b7;

            sleep(250);


        /*    //dcLedOut.setState(!(photoR.getVoltage()<.9));
            ledOut.enable(!(photoR.getVoltage()<.9));
*/
        }
    }
}
