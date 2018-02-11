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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
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

            RelicRecoveryVuMark side = cryptographReader();

            robot.glyphGrab.setPosition(robot.GLYPH_GRAB_CLOSE);
            robot.moveLift(-1000);

            switch (autoType) {
                case AutoBlueAudience:
                    jewelKnocker(robot.jewelServoBlue, 2, false);
                    if(!opModeIsActive()){return;}
                    findTapeAudience(false, robot.tapeSensorRight, 3.0);
                    if(!opModeIsActive()){return;}
                    sleep(100);
                    placeThatGlyphAudience(false, side);
                    break;
                case AutoBlueTimer:
                    jewelKnocker(robot.jewelServoBlue, 2, false);
                    if(!opModeIsActive()){return;}
                    findTapeTimer(false, robot.tapeSensorRight, 3.0);
                    if(!opModeIsActive()){return;}
                    sleep(100);
                    placeThatGlyphTimer(false, side);
                    break;
                case AutoRedAudience:
                    jewelKnocker(robot.jewelServoRed, 1, true);
                    if(!opModeIsActive()){return;}
                    findTapeAudience(true, robot.tapeSensorLeft, 3.0);
                    if(!opModeIsActive()){return;}
                    sleep(100);
                    placeThatGlyphAudience(true, side);
                    break;
                case AutoRedTimer:
                    jewelKnocker(robot.jewelServoRed, 1, true);
                    if(!opModeIsActive()){return;}
                    findTapeTimer(true, robot.tapeSensorLeft, 3.0);
                    if(!opModeIsActive()){return;}
                    sleep(100);
                    placeThatGlyphTimer(true, side);
                    break;
            }
    }


    public RelicRecoveryVuMark cryptographReader() {
        int attempts = 0;
        RelicRecoveryVuMark vuMark;
        do {
            sleep(50);
            vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);
         } while (opModeIsActive() && (attempts++ < 100) && (vuMark == RelicRecoveryVuMark.UNKNOWN));
        RobotLog.ii("TC18_TARGET", String.format("%s visible after %d attempts", vuMark, attempts));
        telemetry.addData("VuMark", "%s visible after %d attempts", vuMark, attempts);
        telemetry.update();
        return vuMark;
    }


    public void jewelKnocker(Servo allianceServo, int port, boolean alliance) {
        double hues[] = new double[6];

        double position = 0.25;
        for (int i = 0; i < 6; i++) {
            allianceServo.setPosition(position);
            sleep(100);
            double hue = robot.colorSensor.getHue(port);
            RobotLog.ii("TC18_JEWEL", String.format("Hue(%d)=%f", i, hue));
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
            robot.spinTurn(2.0, 0.15, 0.5);
            retractBopper(allianceServo);
            sleep(60);
            robot.spinTurn(-2.0, 0.15, 0.5);
        }
        if (alliance && (color == 2)) {
            robot.spinTurn(-2.0, 0.15, 0.5);
            retractBopper(allianceServo);
            sleep(60);
            robot.spinTurn(2.0, 0.15, 0.5);
        }
        if (alliance && (color == 0)) {
            retractBopper(allianceServo);
        }
        if ((!alliance) && (color == 2)) {
            robot.spinTurn(-2.0, 0.15, 0.5);
            retractBopper(allianceServo);
            sleep(60);
            robot.spinTurn(2.0, 0.15, 0.5);
        }
        if ((!alliance) && (color == 1)) {
            robot.spinTurn(2.0, 0.15, 0.5);
            retractBopper(allianceServo);
            sleep(60);
            robot.spinTurn(-2.0, 0.15, 0.5);
        }
        if ((!alliance) && (color == 0)) {
            retractBopper(allianceServo);
        }
    }


    public void retractBopper(Servo allianceServo) {
        allianceServo.setPosition(robot.JEWEL_UP);
        sleep(750);
    }




    public void findTapeAudience(boolean alliance, int hitFirst, double timeout) {
        boolean isColor = false;
        robot.moveThatRobot(0.3, 28, 7.0);
        sleep(200);
        robot.runtime.reset();
        if (alliance) {
            robot.spinTurn(-90, 0.2, 3.1);
            robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.fr.setPower(0.3);
            robot.br.setPower(-0.3);
            robot.fl.setPower(-0.3);
            robot.bl.setPower(0.3);
            while (!isColor && (robot.runtime.seconds() < timeout)) {
                sleep(70);
                double hue = robot.colorSensor.getHue(hitFirst);
                if (hue >= robot.redMin && hue <= robot.redMax) {
                    isColor = true;
                }

            }
            robot.setAllPowers(0.0);
        } else if (!alliance) {
            robot.spinTurn(90, 0.2, 3.1);
            robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.fr.setPower(-0.3);
            robot.br.setPower(0.3);
            robot.fl.setPower(0.3);
            robot.bl.setPower(-0.3);
            while (!isColor && (robot.runtime.seconds() < timeout)) {
                sleep(70);
                double hue = robot.colorSensor.getHue(hitFirst);
                if (hue >= robot.blueMin && hue <= robot.blueMax) {
                    isColor = true;
                }
            }
            robot.setAllPowers(0.0);
        }
        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void findTapeTimer(boolean alliance, int hitFirst, double timeout) {
        boolean isColor = false;
        robot.moveThatRobot(0.5, 33, 7.0);
        robot.runtime.reset();
        robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (alliance) {
            robot.fr.setPower(0.3);
            robot.br.setPower(-0.3);
            robot.fl.setPower(-0.3);
            robot.bl.setPower(0.3);
            while (!isColor && (robot.runtime.seconds() < timeout)) {
                sleep(70);
                double hue = robot.colorSensor.getHue(hitFirst);
                if (hue >= robot.redMin && hue <= robot.redMax) {
                    isColor = true;
                }
            }
            robot.setAllPowers(0.0);
            robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (!alliance) {
            robot.fr.setPower(-0.3);
            robot.br.setPower(0.3);
            robot.fl.setPower(0.3);
            robot.bl.setPower(-0.3);
            while (!isColor && (robot.runtime.seconds() < timeout)) {
                sleep(70);
                double hue = robot.colorSensor.getHue(hitFirst);
                if (hue >= robot.blueMin && hue <= robot.blueMax) {
                    isColor = true;
                }
            }
            robot.setAllPowers(0.0);
            robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void placeThatGlyphAudience(boolean alliance, RelicRecoveryVuMark side) {
        if (alliance) {
            if (side == RelicRecoveryVuMark.RIGHT || side == RelicRecoveryVuMark.UNKNOWN) {
                //robot.strafeThatRobot(0.7, 4.5, 5.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 6.0, 7.0);
            } else if (side == RelicRecoveryVuMark.CENTER) {
                robot.strafeThatRobot(0.7, -4.0, 7.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 8.0, 7.0);
            } else if (side == RelicRecoveryVuMark.LEFT) {
                robot.strafeThatRobot(0.7, -13.5, 8.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.6, 9.0, 7.0);
            }
            robot.glyphGrab.setPosition(robot.GLYPH_GRAB_OPEN);
            sleep(50);
            robot.glyphPush.setPosition(robot.GLYPH_PUSH_OUT);
            robot.moveThatRobot(0.4, -4.0, 6.0);
            robot.glyphPush.setPosition(robot.GLYPH_PUSH_IN);
        } else if (!alliance) {
            if (side == RelicRecoveryVuMark.LEFT || side == RelicRecoveryVuMark.UNKNOWN) {
                //robot.strafeThatRobot(0.7, -4.5, 5.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 7.0, 7.0);
            } else if (side == RelicRecoveryVuMark.CENTER) {
                robot.strafeThatRobot(0.7, 4.0, 7.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 8.0, 7.0);
            } else if (side == RelicRecoveryVuMark.RIGHT) {
                robot.strafeThatRobot(0.7, 13.5, 7.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 9.0, 7.0);
            }
            robot.glyphGrab.setPosition(robot.GLYPH_GRAB_OPEN);
            sleep(50);
            robot.glyphPush.setPosition(robot.GLYPH_PUSH_OUT);
            robot.moveThatRobot(0.4, -4.0, 6.0);
            robot.glyphPush.setPosition(robot.GLYPH_PUSH_IN);
        }
    }

    public void placeThatGlyphTimer(boolean alliance, RelicRecoveryVuMark side) {
        if (alliance) {
            if (side == RelicRecoveryVuMark.RIGHT || side == RelicRecoveryVuMark.UNKNOWN) {
                robot.strafeThatRobot(0.7, 1.0, 5.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 5.0, 7.0);
            } else if (side == RelicRecoveryVuMark.CENTER) {
                robot.strafeThatRobot(0.7, -7.0, 5.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 3.0, 7.0);
            } else if (side == RelicRecoveryVuMark.LEFT) {
                robot.strafeThatRobot(0.7, -15.0, 5.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.6, 6.0, 7.0);
            }
            robot.glyphGrab.setPosition(robot.GLYPH_GRAB_OPEN);
            sleep(50);
            robot.glyphPush.setPosition(robot.GLYPH_PUSH_OUT);
            robot.moveThatRobot(0.4, -4.0, 6.0);
            robot.glyphPush.setPosition(robot.GLYPH_PUSH_IN);
        } else if (!alliance) {
            if (side == RelicRecoveryVuMark.LEFT || side == RelicRecoveryVuMark.UNKNOWN) {
                robot.strafeThatRobot(0.7, -1.0, 5.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 5.0, 7.0);
            } else if (side == RelicRecoveryVuMark.CENTER) {
                robot.strafeThatRobot(0.7, 7.0, 5.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 3.0, 7.0);
            } else if (side == RelicRecoveryVuMark.RIGHT) {
                robot.strafeThatRobot(0.7, 15.0, 5.0);
                robot.moveLift(1000);
                sleep(50);
                robot.moveThatRobot(0.4, 6.0, 7.0);
            }
            robot.glyphGrab.setPosition(robot.GLYPH_GRAB_OPEN);
            sleep(50);
            robot.glyphPush.setPosition(robot.GLYPH_PUSH_OUT);
            robot.moveThatRobot(0.4, -4.0, 6.0);
            robot.glyphPush.setPosition(robot.GLYPH_PUSH_IN);
        }
    }


}


