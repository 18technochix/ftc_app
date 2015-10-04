package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Techno Team_PC_III on 9/27/2015.
 */
public class LegacySensorTest extends LinearOpMode {

    TouchSensor touchy;
    String touchyName = "touchy";
    LightSensor blinky;
    String blinkyName = "blinky";

    public void runOpMode() throws InterruptedException{

        touchy = hardwareMap.touchSensor.get(touchyName);
        blinky = hardwareMap.lightSensor.get(blinkyName);

        waitForStart();

        while(opModeIsActive()){

            //blinky.enableLed(true);
            telemetry.addData("isPressed", String.valueOf(touchy.isPressed()));
            telemetry.addData("color", String.valueOf(blinky.getLightDetected()));

        }

    }


}
