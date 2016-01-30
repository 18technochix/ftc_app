package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.github.ImperialRobotics.BoschIMU.AdafruitIMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;

/**
 * Created by Techno Team_PC_III on 1/30/2016.
 */
public class IMULinearOp extends LinearOpMode {

    AdafruitIMU gyro;

    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            gyro = new AdafruitIMU(hardwareMap, "gyro"

                    //The following was required when the definition of the "I2cDevice" class was incomplete.
                    //, "cdim", 5

                    , (byte)(AdafruitIMU.BNO055_ADDRESS_A * 2)//By convention the FTC SDK always does 8-bit I2C bus
                    //addressing
                    , (byte)AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e){
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
        }

        waitForStart();

        gyro.startIMU();

        while(opModeIsActive()){

            gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
            telemetry.addData("Headings(yaw): ",
                    String.format("Euler= %4.5f, Quaternion calculated= %4.5f", yawAngle[0], yawAngle[1]));
            telemetry.addData("Pitches: ",
                    String.format("Euler= %4.5f, Quaternion calculated= %4.5f", pitchAngle[0], pitchAngle[1]));

        }


    }
}
