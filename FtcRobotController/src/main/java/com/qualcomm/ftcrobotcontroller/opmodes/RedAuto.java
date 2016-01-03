package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Techno Team_PC_III on 11/22/2015.
 */
public class RedAuto extends LinearOpMode {

    Robot robot = new Robot(true, true);

    public void runOpMode() throws InterruptedException{

        robot.initialize(this);

        waitForStart();

        if(opModeIsActive()){

          //robot.sense();
          robot.move(this, 96);

        }


    }


}
