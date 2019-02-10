package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto After Sample", group = "Autonomous")
public class AutoAfterSample extends LinearOpMode {

    private ElapsedTime pressedTime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime newTime = new ElapsedTime();

    //String for color seen. None=0; Red=1; Blue=2; Black=3;
    int position = 1;


    Hardware robot = new Hardware();

    //creating an array for the hue, saturation, and values information
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // to amplify/attentuate the raw RGB measured values.
    final double SCALE_FACTOR = 255;

    @Override
    public void runOpMode() {
        robot.autoInit(hardwareMap);
        waitForStart();
        runtime.reset();
        //while (opModeIsActive()) {
        runtime.reset();
        //covert RGB values to HSV values

        getColor();
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (position == 2) {
            middlePosition();
        } else if (position == 1 || position == 3) {
            while ((robot.touchSensor.isPressed() == false) && robot.backup == 0 && (opModeIsActive())) {
                lineFollow();
                sleep(20);
                getColor();
            }
            if ((robot.touchSensor.isPressed() == true) && opModeIsActive()) {
                telemetry.addLine("Touch Sensor: Pressed");
                telemetry.update();
                pressedTime.reset();
                robot.backup = 1;
                while (opModeIsActive() && robot.backup == 1) {

                    while ((robot.touchSensor.isPressed() == true) && (pressedTime.seconds() <= 3) && (opModeIsActive())) {

                    }
                    if (robot.touchSensor.isPressed() == false) {
                        robot.backup = 0;
                    } else if (pressedTime.seconds() >= 3 && (opModeIsActive())) {
                        telemetry.addData("Starting:", "finish auto");
                        telemetry.update();
                        if (position == 3) {
                            finishAuto3();
                        } else if (position == 1) {
                            finishAuto1();
                        }
                    }
                }
            }
        }
        telemetry.update();
    }

    //line following
    void lineFollow() {
        if (robot.color == 3) {
            robot.leftDrive.setPower(-0.5);
            robot.rightDrive.setPower(0.0);
            telemetry.addLine("Position: Moving left side");
            telemetry.update();
        } else if (robot.color == 1 || robot.color == 2) {
            robot.rightDrive.setPower(-0.5);
            robot.leftDrive.setPower(0.0);
            telemetry.addLine("Position: moving right side");
            telemetry.update();
        } else if (robot.color == 4) {
            robot.rightDrive.setPower(-0.35);
            robot.leftDrive.setPower(0.0);
            telemetry.addLine("Position: Twitching");
            telemetry.update();
        } else {
            robot.leftDrive.setPower(0.0);
            robot.rightDrive.setPower(0.0);
            telemetry.addData("Moving:", "No");
            telemetry.update();
        }
    }

    void finishAuto3() {
        moveThatRobot(.4, 6, 6, 20);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate(1050, .75, 25);
        robot.collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newTime.reset();
        while (opModeIsActive() && newTime.seconds() <= 2) {
            robot.collector.setPower(-0.75);
        }
        moveThatRobot(0.7, -55, -55, 30);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void finishAuto1() {
        moveThatRobot(.4, 6, 6, 20);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate(350, .75, 25);
        robot.collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newTime.reset();
        while (opModeIsActive() && newTime.seconds() <= 2) {
            robot.collector.setPower(-0.75);
        }
        moveThatRobot(0.7, -55, -55, 30);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void finishAuto2() {
        moveThatRobot(.4,4,4,20);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate(1000, .75, 25);
        robot.collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newTime.reset();
        while (opModeIsActive() && newTime.seconds() <= 2) {
            robot.collector.setPower(-0.75);
        }
        moveThatRobot(0.7, -57, -57, 30);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void rotate(double degrees, double speed, double timeoutS) {
        //arc length/2pir = degrees
        //degrees/2pir = arc length
        double circumference = 2 * Math.PI * 6.625;
        double inches = degrees / circumference;

        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        if (degrees > 0) {

            robot.leftDrive.setTargetPosition(-newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
        } else if (degrees < 0) {
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(-newRightTarget);
        }
        // Turn On RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("update:", "about to turn on motors");
        telemetry.update();
        robot.leftDrive.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void getColor() {

        Color.RGBToHSV((int) (robot.colorSensor.red() * SCALE_FACTOR),
                (int) (robot.colorSensor.green() * SCALE_FACTOR),
                (int) (robot.colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        //setting number values for each color
        if ((hsvValues[0] <= 360 && hsvValues[0] >= 275) || (hsvValues[0] >= 0 && hsvValues[0] <= 10)) {
            robot.color = 1;
            telemetry.addData("Color:", "Red");
        } else if (hsvValues[0] >= 175 && hsvValues[0] <= 250) {
            robot.color = 2;
            telemetry.addData("Color:", "Blue");
        } else if (hsvValues[0] <= 125 && hsvValues[0] >= 60) {
            robot.color = 3;
            telemetry.addData("Color:", "Black");
        } else if ((hsvValues[0] >= 130 && hsvValues[0] <= 170) || (hsvValues[0] <= 30 && hsvValues[0] >= 20)) {
            robot.color = 4;
            telemetry.addData("Color:", "Middle color");
        } else {
            robot.color = 0;
            telemetry.addData("Color:", hsvValues[0]);
            telemetry.update();
        }
    }

    public void moveThatRobot(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        runtime.reset();
        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
        newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);
        robot.leftDrive.setTargetPosition(newLeftTarget);
        robot.rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("update:", "about to turn on motors");
        telemetry.update();
        robot.leftDrive.setPower(Math.abs(speed));
        robot.rightDrive.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    public void middlePosition() {
        while ((robot.touchSensor.isPressed() == false) && robot.backup == 0 && (opModeIsActive())) {
            robot.rightDrive.setPower(0.4);
            robot.leftDrive.setPower(0.4);
        }
        if ((robot.touchSensor.isPressed() == true) && opModeIsActive()) {
            telemetry.addLine("Touch Sensor: Pressed");
            telemetry.update();
            pressedTime.reset();
            robot.backup = 1;
            while (opModeIsActive() && robot.backup == 1) {

                while ((robot.touchSensor.isPressed() == true) && (pressedTime.seconds() <= 3) && (opModeIsActive())) {

                }
                if (robot.touchSensor.isPressed() == false) {
                    robot.backup = 0;
                } else if (pressedTime.seconds() >= 3 && (opModeIsActive())) {
                    telemetry.addData("Starting:", "finish auto");
                    telemetry.update();
                    finishAuto2();
                }
            }
        }
    }
}