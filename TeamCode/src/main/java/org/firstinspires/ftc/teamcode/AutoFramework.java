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

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.autoInit(hardwareMap);

        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        double knockDistance = 0.;

        switch (autoType) {
            case AutoBlueAudience:
                robot.glyphGrab.setPosition(robot.GLYPH_GRAB_CLOSE);
                robot.moveLift(-1000);
                knockDistance = jewelKnocker(robot.jewelServoBlue, 2, false);
                glyphPlaceAudience(false, knockDistance);
                robot.moveThatRobot(0.6 * (1.0 + 12.5/70.0), 0.6, -5.0, 5.0);
                break;
            case AutoBlueTimer:
                robot.glyphGrab.setPosition(robot.GLYPH_GRAB_CLOSE);
                robot.moveLift(-1000);
                knockDistance = jewelKnocker(robot.jewelServoBlue, 2, false);
                placeGlyphTimer(false, knockDistance);
                robot.moveThatRobot(0.6 * (1.0 + 12.5/70.0), 0.6, -5.0, 5.0);
                break;
            case AutoRedAudience:
                robot.glyphGrab.setPosition(robot.GLYPH_GRAB_CLOSE);
                robot.moveLift(-1000);
                knockDistance = jewelKnocker(robot.jewelServoRed, 1, true);
                glyphPlaceAudience(true, knockDistance);
                robot.moveThatRobot(0.6 * (1.0 + 12.5/70.0), 0.6, -5.0, 5.0);
                break;
            case AutoRedTimer:
                robot.glyphGrab.setPosition(robot.GLYPH_GRAB_CLOSE);
                robot.moveLift(-1000);
                knockDistance = jewelKnocker(robot.jewelServoRed, 1, true);
                placeGlyphTimer(true, knockDistance);
                robot.moveThatRobot(0.6 * (1.0 + 12.5/70), 0.6, -5.0, 5.0);
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

        double position = 0.25;
        for (int i = 0; i < 6; i++) {
            allianceServo.setPosition(position);
            sleep(50);
            double hue = robot.colorSensor.getHue(port);
            telemetry.addData("Status", "Hue: " + hue);
            telemetry.update();
            position -= 0.05;
            Array.set(hues, i, hue);
        }
        int color = 0;
        for (int j = 0; j < 6; j++) {
            if (hues[j] > robot.redMin && hues[j] < robot.redMax) {
                color = 1;
            } else if (hues[j] > robot.blueMin && hues[j] < robot.blueMax) {
                color = 2;
            }
        }
        if (alliance && (color == 1)) {
            robot.moveThatRobot(0.5, 2.0, 1.0);
            retractBopper(allianceServo);
            return -2.0;
        }
        if (alliance && (color == 2)) {
            robot.moveThatRobot(0.5, -2.0, 1.0);
            retractBopper(allianceServo);
            return 2.0;
        }
        if (alliance && (color == 0)) {
            retractBopper(allianceServo);
            return 0.0;
        }
        if ((!alliance) && (color == 2)) {
            robot.moveThatRobot(0.5, 2.0, 1.0);
            retractBopper(allianceServo);
            return -2.0;
        }
        if ((!alliance) && (color == 1)) {
            robot.moveThatRobot(0.5, -2.0, 1.0);
            retractBopper(allianceServo);
            return 2.0;
        }
        if ((!alliance) && (color == 0)) {
            retractBopper(allianceServo);
            return 0.0;
        }
        return 0.0;
    }


    public void retractBopper(Servo allianceServo) {
        allianceServo.setPosition(robot.JEWEL_UP);
        sleep(750);
    }


    public void glyphPlaceAudience(boolean alliance, double distance){
        robot.moveThatRobot(0.6 * (1.0 + 12.5/70.0), 0.6, 38 + distance, 5.0);
            if(alliance){
                robot.fr.setPower(-0.5);
                robot.fl.setPower(0.5);
                robot.br.setPower(-0.5);
                robot.bl.setPower(0.5);
                sleep(1250);
            } else if(!alliance){
                robot.fr.setPower(0.5);
                robot.fl.setPower(-0.5);
                robot.br.setPower(0.5);
                robot.bl.setPower(-0.5);
                sleep(1250);
            }
        robot.fr.setPower(0.0);
        robot.fl.setPower(0.0);
        robot.br.setPower(0.0);
        robot.bl.setPower(0.0);
        robot.moveThatRobot(0.6 * (1.0 + 12.5/70.0), 0.6, 10, 5.0);
        robot.moveLift(1000);
        robot.glyphGrab.setPosition(robot.GLYPH_GRAB_OPEN);
        robot.moveThatRobot(0.6 * (1.0 + 12.5/70.0), 0.6, 2, 5.0);
    }

    public void placeGlyphTimer(boolean alliance, double distance){
        if(alliance){
            robot.moveThatRobot(0.6 * (1.0 + 12.5/70.0), 0.6, 36 + distance, 5.0);
            robot.fr.setPower(0.5);
            robot.br.setPower(-0.5);
            robot.fl.setPower(-0.5);
            robot.bl.setPower(0.5);
        } else if(!alliance){
            robot.moveThatRobot(0.6 * (1.0 + 12.5/70.0), 0.6, 33 + distance, 5.0);
            robot.fr.setPower(-0.5);
            robot.br.setPower(0.5);
            robot.fl.setPower(0.5);
            robot.bl.setPower(-0.5);
        }
        sleep(1000);
        robot.fr.setPower(0.0);
        robot.fl.setPower(0.0);
        robot.br.setPower(0.0);
        robot.bl.setPower(0.0);
        robot.moveLift(1000);
        robot.glyphGrab.setPosition(robot.GLYPH_GRAB_OPEN);
        robot.moveThatRobot(0.6 * (1.0 + 12.5/70.0), 0.6, 3, 5.0);
    }


    //two sides of the field: left = the pair of red and blue where the are across from eachother, and the pair where the are next to eachother
    //side by side: true
    //next to eachother: false
       /* public void findTape(boolean side, boolean alliance, RelicRecoveryVuMark direction) {
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

    // true side = side where the red and blue are next to eachother
    //true alliance = blue
    public void tapeFinder(boolean alliance, boolean side) {
        //array of the colors of which the sensors pick up
        double rightColor = robot.colorSensor.getHue(robot.tapeSensorRight);
        double leftColor = robot.colorSensor.getHue(robot.tapeSensorLeft);
        double tapeColors[] = {rightColor, leftColor};
        //  boolean

        if (alliance && side) {
            //we are now facing the jewls, and for the sensors to be facing the cryptobox, the robot has to turn 180. But lets first roll of the balancing stone so its easier
            robot.moveThatRobotSide(0.5, 0.5, 0.5, 0.5, 18, 2);
            //now lets spin the robot to face it to the cryptobox
            robot.spinRobot(180, 0.5);


        }

    }

    public void centerOnTape(boolean alliance) {
        boolean LeftreadingBlue = false;
        boolean LeftreadingRed = false;
        boolean RightreadingBlue = false;
        boolean RightreadingRed = false;
        double colorOfLeft = robot.colorSensor.getHue(robot.tapeSensorLeft);
        double colorOfRight = robot.colorSensor.getHue(robot.tapeSensorRight);
        if (colorOfLeft > robot.blueMin && colorOfLeft < robot.blueMax) {
            LeftreadingBlue = true;
        }
        if (colorOfLeft > robot.redMin && colorOfLeft < robot.redMax) {
            LeftreadingRed = true;
        }
        if (colorOfRight > robot.blueMin && colorOfRight < robot.blueMax) {
            RightreadingBlue = true;
        }
        if (colorOfRight > robot.redMin && colorOfRight < robot.redMax) {
            RightreadingRed = true;
        }

        if (alliance) {
            if (LeftreadingBlue == true && RightreadingBlue == false) {
                while (LeftreadingBlue == true && RightreadingBlue == false) {
                    telemetry.addData("LeftColor", colorOfLeft);
                    telemetry.addData("RightColor",colorOfRight);
                    telemetry.update();
                    //moveLeft();
                }
            } else if (RightreadingBlue == true && LeftreadingBlue == true) {
                while (RightreadingBlue == true && LeftreadingBlue == true) {
                    telemetry.addData("LeftColor", colorOfLeft);
                    telemetry.addData("RightColor",colorOfRight);
                    telemetry.update();
                    //moveRight();

                }
            } else if (LeftreadingBlue == false && RightreadingBlue == false) ;
            {
                while (LeftreadingBlue == false && RightreadingBlue == false) {
                    telemetry.addData("LeftColor", colorOfLeft);
                    telemetry.addData("RightColor",colorOfRight);
                    telemetry.update();
                    robot.fr.setPower(0.5);
                    robot.fl.setPower(0.5);
                    robot.br.setPower(0.5);
                    robot.bl.setPower(0.5);
                }
            }
        } else if (!alliance) {
            if (LeftreadingRed == true && RightreadingRed == false) {
                while (LeftreadingRed == true && RightreadingRed == false) {
                    telemetry.addData("LeftColor", colorOfLeft);
                    telemetry.addData("RightColor",colorOfRight);
                    telemetry.update();
                    //moveLeft();
                }
            } else if (RightreadingRed == true && LeftreadingRed == true) {
                while (RightreadingRed == true && LeftreadingRed == true) {
                    telemetry.addData("LeftColor", colorOfLeft);
                    telemetry.addData("RightColor",colorOfRight);
                    telemetry.update();
                    //moveRight();
                }
            } else if (LeftreadingRed == false && RightreadingRed == false) ;
            {
                while (LeftreadingRed == false && RightreadingRed == false) {
                    telemetry.addData("LeftColor", colorOfLeft);
                    telemetry.addData("RightColor",colorOfRight);
                    telemetry.update();
                    robot.fr.setPower(0.5);
                    robot.fl.setPower(0.5);
                    robot.br.setPower(0.5);
                    robot.bl.setPower(0.5);
                }

            }


        }
    }
    */

}



