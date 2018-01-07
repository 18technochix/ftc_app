package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 11/30/2017.
 */

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.lang.reflect.Array;

@Autonomous(name="Auto Framework", group="Autonomous")
@Disabled
public class AutoFramework extends LinearOpMode {
    enum AutoType {AutoRedTimer, AutoRedAudience, AutoBlueTimer, AutoBlueAudience}

    AutoType autoType;
    Hardware robot = new Hardware(this);


    //Initialize all of your motors, servos, sensors

    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    //private ElapsedTime     runtime = new ElapsedTime();

    //Place constants here e.g. encoder counts!

    //Here's where the magic actually happens lol
    public void runOpMode() {
        //Here's an example of telemetry, this displays on the phone
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //init call from Hardware class
        robot.autoInit(hardwareMap);

        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        double knockDistance = 0.;

        switch (autoType) {
            case AutoBlueAudience:
                knockDistance = jewelKnocker(robot.jewelServoBlue, 2, false);
                //park(35 + knockDistance, 0 , 0);
                break;
            case AutoBlueTimer:
                knockDistance = jewelKnocker(robot.jewelServoBlue, 2, false);
                // park(35 + knockDistance, 0 /*90*/ , 0);
                break;
            case AutoRedAudience:
                knockDistance = jewelKnocker(robot.jewelServoRed, 1,true);
                //  park(35 + knockDistance, 0 , 0);
                break;
            case AutoRedTimer:
                knockDistance = jewelKnocker(robot.jewelServoRed, 1, true);
                //  park(35 + knockDistance, 0 /*-90*/ , 0);
                break;
        }
    }



     public RelicRecoveryVuMark crytographReader() {
     VuforiaTrackables relicTrackables = robot.picReader.loadTrackablesFromAsset("RelicVuMark");
     VuforiaTrackable relicTemplate = relicTrackables.get(0);
     relicTemplate.setName("relicVuMarkTemplate");
     relicTrackables.activate();
     RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
     if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
         OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
         if (pose != null) {
             VectorF trans = pose.getTranslation();
             Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
             double tX = trans.get(0);
             double tY = trans.get(1);
             double tZ = trans.get(2);
             double rX = rot.firstAngle;
             double rY = rot.secondAngle;
             double rZ = rot.thirdAngle;
         }
     }
     RelicRecoveryVuMark side = vuMark;
     telemetry.addData("VuMark", "%s visible", side);
     telemetry.update();
     return side;
     }


    public double jewelKnocker(Servo allianceServo, int port, boolean alliance) {
        double hues[] = new double[6];
        for (int i = 0; i < 6; i++) {
            double position = 0.25;
            allianceServo.setPosition(position);
            double hue = robot.colorSensor.getHue(port);
            position -= 0.05;
            Array.set(hues, i, hue);
        }
        int color = 0;
        for (int j = 0; j < 6; j++) {
            if (hues[j] > robot.redMin && hues[j] < robot.redMax) {
                color = 1;
            }
            if (hues[j] > robot.blueMin && hues[j] < robot.blueMax) {
                color = 2;
            }
        }
            if (alliance && color == 1) {
                robot.jewelServoRed.setPosition(1.0);
                robot.moveThatRobot(0.5, 2.0, 1.0);
                retractBopper(allianceServo);
                return -2.0;
            }
            if (alliance && color == 2) {
                robot.jewelServoRed.setPosition(1.0);
                robot.moveThatRobot(0.5, -2.0, 1.0);
                retractBopper(allianceServo);
                return 2.0;
            }
            if (alliance && color == 0) {
                retractBopper(allianceServo);
                return 0.0;
            }
            if (!alliance && color == 2) {
                robot.jewelServoBlue.setPosition(0.5);
                robot.moveThatRobot(0.5, 2.0, 1.0);
                retractBopper(allianceServo);
                return -2.0;
            }
            if (!alliance && color == 1) {
                robot.jewelServoBlue.setPosition(0.5);
                robot.moveThatRobot(0.5, -2.0, 1.0);
                retractBopper(allianceServo);
                return 2.0;
            }
            if (!alliance && color == 0) {
                retractBopper(allianceServo);
                return 0.0;
            }
            return 0.0;
    }


    public void retractBopper(Servo allianceServo){
        allianceServo.setPosition(robot.JEWEL_UP);
        sleep(750);
    }


        public void park(double distance){
            robot.moveThatRobot(0.5, distance, 15.0);
            sleep(50);
            robot.fr.setPower(0.0);
            robot.br.setPower(0.0);
            robot.fl.setPower(0.0);
            robot.bl.setPower(0.0);

        }
//two sides of the field: left = the pair of red and blue where the are across from eachother, and the pair where the are next to eachother
    //side by side: true
    //next to eachother: false
        public void findTape(boolean side, boolean alliance, RelicRecoveryVuMark direction) {
            double hue1 = robot.colorSensor.getHue(robot.tapeSensor1Port);
            double hue2 = robot.colorSensor.getHue(robot.tapeSensor2Port);
            boolean isColor = false;
            if (side) {
                while (!isColor) {
                    if ((hue2 >= robot.blueMin && hue2 <= robot.blueMin) && (hue2 >= robot.redMin && hue2 <= robot.redMin)) {
                        isColor = true;
                    }
                    robot.fr.setPower(0.1);
                    robot.br.setPower(0.1);
                    robot.bl.setPower(0.1);
                    robot.fl.setPower(0.1);
                }
                if (alliance) {
                    while(!(hue1 >= robot.redMin && hue1 <= robot.redMin)){
                       robot.fr.setPower(0.2);
                       robot.fl.setPower(0.2);
                    }
                    robot.spinRobot(90, 0.2);
                    robot.moveThatRobot(0.5, 7.0, 10.0);
                } else if(!alliance) {
                    while(!(hue1 >= robot.blueMin && hue1 <= robot.blueMin)){
                        robot.fr.setPower(-0.2);
                        robot.fl.setPower(-0.2);
                    }
                    robot.spinRobot(-90, 0.2);
                    robot.moveThatRobot(0.5, 7.0, 10.0);
                }
                if(direction == RelicRecoveryVuMark.CENTER || direction == RelicRecoveryVuMark.UNKNOWN){
                    robot.moveThatRobot(0.1, 3.0, 5.0);
                }
                if(direction == RelicRecoveryVuMark.LEFT){
                    robot.moveThatRobotSide(-0.5, 0.5, 0.5, -0.5, 3.0, 10.0);
                    robot.moveThatRobot(0.1, 3.0, 5.0);
                }
                if(direction == RelicRecoveryVuMark.RIGHT){
                    robot.moveThatRobotSide(0.5, -0.5, -0.5, 0.5, 3.0, 10.0);
                    robot.moveThatRobot(0.1, 3.0, 5.0);
                }
            }
        }

   public void placeGlyph(){
        //lower lift
        robot.glyphGrab.setPosition(robot.RELIC_GRAB_OPEN);


}




}