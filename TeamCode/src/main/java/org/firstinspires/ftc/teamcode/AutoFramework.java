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


            double knockDistance = 0.;
            sleep(500);
            RelicRecoveryVuMark side = cryptographReader();

            robot.glyphGrab.setPosition(robot.GLYPH_GRAB_CLOSE);
            robot.moveLift(-1000);

            switch (autoType) {
                case AutoBlueAudience:
                    knockDistance = jewelKnocker(robot.jewelServoBlue, 2, false);
                    findTapeAudience(false, robot.tapeSensorRight, knockDistance);
                    placeThatGlyph(false, side);
                    break;
                case AutoBlueTimer:
                    knockDistance = jewelKnocker(robot.jewelServoBlue, 2, false);
                    findTapeTimer(false, robot.tapeSensorRight, knockDistance);
                    placeThatGlyph(false, side);
                    break;
                case AutoRedAudience:
                    knockDistance = jewelKnocker(robot.jewelServoRed, 1, true);
                    findTapeAudience(true, robot.tapeSensorLeft, knockDistance);
                    placeThatGlyph(true, side);
                    break;
                case AutoRedTimer:
                    knockDistance = jewelKnocker(robot.jewelServoRed, 1, true);
                    findTapeTimer(true, robot.tapeSensorLeft, knockDistance);
                    placeThatGlyph(true, side);
                    break;
            }
    }


    public RelicRecoveryVuMark cryptographReader() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);
        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.update();
        return vuMark;
    }


    public double jewelKnocker(Servo allianceServo, int port, boolean alliance) {
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
            //robot.moveThatRobot(0.5, 3.0, 1.0);
            //return -3.0;
            robot.spinTurn(2.5, 0.15);
            retractBopper(allianceServo);
            sleep(60);
            robot.spinTurn(-2.5, 0.15);
        }
        if (alliance && (color == 2)) {
            //robot.moveThatRobot(0.5, -3.0, 1.0);
            //return 5.0;
            robot.spinTurn(-2.5, 0.15);
            retractBopper(allianceServo);
            sleep(60);
            robot.spinTurn(2.5, 0.15);
        }
        if (alliance && (color == 0)) {
            retractBopper(allianceServo);
            return 0.0;
        }
        if ((!alliance) && (color == 2)) {
            //robot.moveThatRobot(0.5, 3.0, 1.0);
            //return -3.0;
            robot.spinTurn(-2.5, 0.15);
            retractBopper(allianceServo);
            sleep(60);
            robot.spinTurn(2.5, 0.15);
        }
        if ((!alliance) && (color == 1)) {
            robot.spinTurn(2.5, 0.15);
            retractBopper(allianceServo);
            sleep(60);
            robot.spinTurn(-2.5, 0.15);
            //robot.moveThatRobot(0.5, -3.0, 1.0);
            //return 5.0;
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


    public void glyphPlaceAudience(boolean alliance, double distance) {
        robot.moveThatRobot(0.6, 0.6, 38 + distance, 5.0);
        if (alliance) {
            robot.fr.setPower(-0.5);
            robot.fl.setPower(0.5);
            robot.br.setPower(-0.5);
            robot.bl.setPower(0.5);
            sleep(1250);
        } else if (!alliance) {
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
        robot.moveThatRobot(0.6, 0.6, 10, 5.0);
        robot.moveLift(1000);
        robot.glyphGrab.setPosition(robot.GLYPH_GRAB_OPEN);
        robot.moveThatRobot(0.6, 0.6, 2, 5.0);
    }

    public void placeGlyphTimer(boolean alliance, double distance) {
        if (alliance) {
            robot.moveThatRobot(0.6, 0.6, 36 + distance, 5.0);
            robot.fr.setPower(0.5);
            robot.br.setPower(-0.5);
            robot.fl.setPower(-0.5);
            robot.bl.setPower(0.5);
        } else if (!alliance) {
            robot.moveThatRobot(0.6, 0.6, 33 + distance, 5.0);
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
        robot.moveThatRobot(0.6, 0.6, 3, 5.0);
    }


    public void findTapeAudience(boolean alliance, int hitFirst, double distance) {
        boolean isColor = false;
        robot.moveThatRobot(0.4, 25 + distance, 7.0);
        sleep(200);
        if (alliance) {
            robot.spinTurn(-90, 0.3);
            robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.fr.setPower(0.3);
            robot.br.setPower(-0.3);
            robot.fl.setPower(-0.3);
            robot.bl.setPower(0.3);
            while (!isColor) {
                sleep(70);
                double hue = robot.colorSensor.getHue(hitFirst);
                if (hue >= robot.redMin && hue <= robot.redMax) {
                    isColor = true;
                }

            }
            robot.setAllPowers(0.0);
        } else if (!alliance) {
            robot.spinTurn(90, 0.3);
            robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.fr.setPower(-0.3);
            robot.br.setPower(0.3);
            robot.fl.setPower(0.3);
            robot.bl.setPower(-0.3);
            while (!isColor) {
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

    public void findTapeTimer(boolean alliance, int hitFirst, double distance) {
        boolean isColor = false;
        robot.moveThatRobot(0.5, 33 + distance, 7.0);
        robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (alliance) {
            robot.fr.setPower(0.3);
            robot.br.setPower(-0.3);
            robot.fl.setPower(-0.3);
            robot.bl.setPower(0.3);
            while (!isColor) {
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
            while (!isColor) {
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

    public void placeThatGlyph(boolean alliance, RelicRecoveryVuMark side) {
        if (alliance) {
            if (side == RelicRecoveryVuMark.RIGHT || side == RelicRecoveryVuMark.UNKNOWN) {
                robot.moveThatRobot(0.4, 8.0, 7.0);
            } else if (side == RelicRecoveryVuMark.CENTER) {
                robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fr.setPower(0.3);
                robot.br.setPower(-0.3);
                robot.fl.setPower(-0.3);
                robot.bl.setPower(0.3);
                sleep(200);
                robot.setAllPowers(0.0);
                robot.moveThatRobot(0.4, 8.0, 7.0);
            } else if (side == RelicRecoveryVuMark.LEFT) {
                robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fr.setPower(0.3);
                robot.br.setPower(-0.3);
                robot.fl.setPower(-0.3);
                robot.bl.setPower(0.3);
                sleep(400);
                robot.setAllPowers(0.0);
                robot.moveThatRobot(0.4, 8.0, 7.0);
            }
            robot.moveLift(1000);
            robot.glyphGrab.setPosition(robot.GLYPH_GRAB_OPEN);
            robot.moveThatRobot(0.4, -4.0, 6.0);
        } else if (!alliance) {
            if (side == RelicRecoveryVuMark.LEFT || side == RelicRecoveryVuMark.UNKNOWN) {
                robot.moveThatRobot(0.4, 8.0, 7.0);
            } else if (side == RelicRecoveryVuMark.CENTER) {
                robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fr.setPower(-0.3);
                robot.br.setPower(0.3);
                robot.fl.setPower(0.3);
                robot.bl.setPower(-0.3);
                sleep(500);
                robot.setAllPowers(0.0);
                robot.moveThatRobot(0.4, 8.0, 7.0);
            } else if (side == RelicRecoveryVuMark.RIGHT) {
                robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.fr.setPower(-0.3);
                robot.br.setPower(0.3);
                robot.fl.setPower(0.3);
                robot.bl.setPower(-0.3);
                sleep(1000);
                robot.setAllPowers(0.0);
                robot.moveThatRobot(0.4, 8.0, 7.0);
            }
            robot.moveLift(1000);
            robot.glyphGrab.setPosition(robot.GLYPH_GRAB_OPEN);
            robot.moveThatRobot(0.4, -4.0, 6.0);
        }
    }


}


