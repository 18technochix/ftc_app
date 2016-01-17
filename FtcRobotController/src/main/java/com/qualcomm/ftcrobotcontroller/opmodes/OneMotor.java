package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Techno Team_PC_III on 1/17/2016.
 */
public class OneMotor extends LinearOpMode {

    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.dcMotor.get("motor");

        waitForStart();

        motor.setPower(0.5);

    }

}
