package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Techno Team_PC_III on 11/22/2015.
 */
public class RedAuto extends RobotOpMode {

    public void runOpMode() throws InterruptedException{

        auto = true;
        red = true;
        super.runOpMode(); //initalizes robot

        waitForStart();

        if(opModeIsActive()){

          //robot.move(this, 96);
            while(followLine() == false){

            }
            sense();

        }


    }


}
