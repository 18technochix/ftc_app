package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Techno Team_PC_III on 9/27/2015.
 */
public class MRTouchTest extends LinearOpMode {

    TouchSensor touchy;



    public void runOpMode() throws InterruptedException {

        touchy = hardwareMap.touchSensor.get("touchy");

        waitForStart();

        while(opModeIsActive()) {
            if(touchy.isPressed()) {
                DbgLog.msg("touchy was touched.");
            } else {
                DbgLog.msg("touchy is not touched.");
            }

            telemetry.addData("isPressed", String.valueOf(touchy.isPressed()));

            // Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }

    }



}
