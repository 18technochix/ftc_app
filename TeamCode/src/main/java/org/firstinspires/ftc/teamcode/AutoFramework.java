package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 11/30/2017.
 */

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="Auto Framework", group="Autonomous")
//@Disabled
public class AutoFramework extends LinearOpMode {

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

        //make calls to the hardware map (this is just a formality, dw bout what it actually do)


        robot.lift= hardwareMap.dcMotor.get("particle lift");

        //ensure everything is going in the direction you want
        robot.fl.setDirection(DcMotor.Direction.REVERSE);
        robot.bl.setDirection(DcMotor.Direction.REVERSE);
        robot.lift.setDirection(DcMotor.Direction.REVERSE);


        //when using encoders, do this, don't ask why (check the yellow postit on the desktop)

        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);


       robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set the initial values for ya servos
        //servos work for values from 0-1, you can use the MR program to determine
        //values from 0-255 and use the corresponding fraction:
    robot.grabServo.setPosition(0.0);
    robot.jewelServoBlue.setPosition(0.0);
    robot.jewelServoRed.setPosition(0.0);


        //set the power for all of your motors to 0. why? because i said so. don't make robo move just yet


        //Here's the ticket yo, everything before this is initialization, and after this is all of
        //waitForStart();
        jewelKnocker(true);
        //putBoxIntoCryptobox(crytographReader());
        //park();


    }

    //METHODS
    public void moveThatRobot(double power, double inches, String direction, double timeout) {
        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double wheelCircumference = 2 * (Math.PI) * 2;
        int revolutions = (int) (inches / wheelCircumference);
        int encoderTicks = 1120 * revolutions;
        int position = robot.fr.getCurrentPosition();
        while (position < encoderTicks) {
            if (direction == "left") {
                robot.fr.setPower(-power);
                robot.br.setPower(power);
                robot.fl.setPower(power);
                robot.bl.setPower(-power);
            } else if (direction == "right") {
                robot.fr.setPower(-power);
                robot.br.setPower(power);
                robot.fl.setPower(power);
                robot.bl.setPower(-power);
            } else if (direction == "forward") {
                robot.fr.setPower(power);
                robot.br.setPower(power);
                robot.fl.setPower(power);
                robot.bl.setPower(power);
            } else if (direction == "back") {
                robot.fr.setPower(-power);
                robot.br.setPower(-power);
                robot.fl.setPower(-power);
                robot.bl.setPower(-power);
            }
        }

        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    //knocks jewel off according to color
    public void jewelKnocker(boolean alliance){
        if (alliance) {
            extendBopper(robot.jewelServoRed);
            int color = this.getColor(robot.jewelSensorRed);
            if (color >= robot.redMin && color <= 360 || color >= 0 && color <= robot.redMax) {
                moveThatRobot(.5, 2, "forward", 0);
            } else {
                moveThatRobot(.5, 2, "back", 0);
            }
            retractBopper(robot.jewelServoRed);
        } else if (!alliance) {
            extendBopper(robot.jewelServoBlue);
            int color = this.getColor(robot.jewelSensorBlue);
            if (color >= robot.blueMin && color <= robot.blueMax) {
                moveThatRobot(.5, 2, "left", 0);
            } else {
                moveThatRobot(.5, 2, "right", 0);
            }
            retractBopper(robot.jewelServoBlue);
        }
    }

    /** public RelicRecoveryVuMark crytographReader() {

        VuforiaTrackables relicTrackables = robot.picReader.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        // yeah fam idk what this does but its in the vuforia code and it won't work bc of the pose variabl
         if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
         OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
         }
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


        RelicRecoveryVuMark side = vuMark;
        return side;
    }


    public void putBoxIntoCryptobox(RelicRecoveryVuMark box) {
        if (box == RelicRecoveryVuMark.RIGHT) {
            moveThatRobot(0.5, 12.0, "right", 0.0);
        } else if (box == RelicRecoveryVuMark.LEFT) {
            moveThatRobot(0.5, 12.0, "left", 0.0);
        }else if(box == RelicRecoveryVuMark.CENTER) {
            moveThatRobot(0.5, 6.0, "right", 0.0);
        }else{
            moveThatRobot(0.0, 0.0, "right", 0.0);
        }

}
     */

    //returns the color of the Jewels or the balancing stone

    public int getColor(ColorSensor sensor){
        float hsvValues[] = {0F,0F,0F};
        float values[] = hsvValues;
        boolean LedOn = true;
        sensor.enableLed(LedOn);
        Color.RGBToHSV((sensor.red() * 255) / 800, (sensor.green() * 255) / 800, (sensor.blue() * 255) / 800, hsvValues);
        telemetry.addData("LED", LedOn ? "On" : "Off");
        telemetry.addData("Clear", sensor.alpha());
        telemetry.addData("Red  ", sensor.red());
        telemetry.addData("Green", sensor.green());
        telemetry.addData("Blue ", sensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        int color =  Color.HSVToColor(0xff, values);

        return color;



    }


    public void extendBopper(Servo allianceServo){
        allianceServo.setPosition(180);
    }

    public void offTheStone(String direction){
        moveThatRobot(0.5, 15, direction,0.0);
    }

    public void retractBopper(Servo allianceServo){
        allianceServo.setPosition(0);
    }
/**
        public void spinRobot(double angle, double power){
            while(robot.getHeading()< angle ) {
                robot.fr.setPower(power);
                robot.br.setPower(power);
                robot.fl.setPower(-power);
                robot.fr.setPower(-power);

            }
            power = 0;
            robot.fr.setPower(power);
            robot.br.setPower(power);
            robot.fl.setPower(-power);
            robot.fr.setPower(-power);

        }
 */
public void readColor(ColorSensor colorSensor){
}
public void park(){
            moveThatRobot(0.0, 0.0, "left", 0.0);
        }

}

