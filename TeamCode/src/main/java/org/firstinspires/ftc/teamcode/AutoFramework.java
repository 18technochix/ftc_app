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

@Autonomous(name="Auto Framework", group="Autonomous")
@Disabled
public class AutoFramework extends LinearOpMode {
    enum AutoType { AutoRedTimer, AutoRedAudience, AutoBlueTimer, AutoBlueAudience }
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

       switch(autoType){
           case AutoBlueAudience:
               knockDistance = jewelKnocker(false);
               park(35 + knockDistance, 0 , 0);
               break;
           case AutoBlueTimer:
               knockDistance = jewelKnocker(false);
               park(35 + knockDistance, 0 /*90*/ , 0);
               break;
           case AutoRedAudience:
               knockDistance = jewelKnocker(true);
               park(35 + knockDistance, 0 , 0);
               break;
           case AutoRedTimer:
               knockDistance = jewelKnocker(true);
               park(35 + knockDistance, 0 /*-90*/ , 0);
               break;
       }


        //Here's the ticket yo, everything before this is initialization, and after this is all of
        //waitForStart();
        //jewelKnocker(true);
        //putBoxIntoCryptobox(crytographReader());
        //park();


    }

    //METHODS
    /**
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
                obot.br.setPower(power);
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
     */

    //knocks jewel off according to color
    public double jewelKnocker(boolean alliance){
        double hue = 0;
        if(alliance) {
            extendBopper(robot.jewelServoRed);
            hue = robot.colorSensor.getHue(robot.redPort);
            telemetry.addData( "color", hue);
            telemetry.update();

            if (hue >= robot.redMin && hue <= robot.redMax) {
               robot.moveThatRobot(0.5, 2.0, 1.0);
                retractBopper(robot.jewelServoRed);
               return -2.0;
            } else if(hue >= robot.blueMin && hue <= robot.blueMax) {
                robot.moveThatRobot(0.5, -2.0, 1.0);
                retractBopper(robot.jewelServoRed);
                return 2.0;
            } else {
                retractBopper(robot.jewelServoRed);
                return 0.0;
            }
        } else if (!alliance) {
            extendBopper(robot.jewelServoBlue);
            hue = robot.colorSensor.getHue(robot.bluePort);
            telemetry.addData( "color", hue);
            telemetry.update();
            if (hue >= robot.blueMin && hue <= robot.blueMax) {
                robot.moveThatRobot(0.5, 2.0, 1.0);
                retractBopper(robot.jewelServoBlue);
                return -2.0;
            } else if(hue >= robot.redMin && hue <= robot.redMax) {
                robot.moveThatRobot(0.5, -2.0, 1.0);
                retractBopper(robot.jewelServoBlue);
                return 2.0;
            } else {
                retractBopper(robot.jewelServoBlue);
                return 0.0;
            }
        }
        return 0.0;
     }
     //check if red or blue is greater

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


    public void extendBopper(Servo allianceServo){
        allianceServo.setPosition(robot.JEWEL_DOWN);
        sleep(1000);
    }


    public void retractBopper(Servo allianceServo){
        allianceServo.setPosition(robot.JEWEL_UP);
        sleep(750);
    }
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
        public void park(double distance1, double angle, double distance2){
            robot.moveThatRobot(0.5, distance1, 15.0);
            //spinRobot(angle, 0.5);
            //robot.moveThatRobot(0.5, distance2, 1.0);
            sleep(50);
            robot.fr.setPower(0.0);
            robot.br.setPower(0.0);
            robot.fl.setPower(0.0);
            robot.bl.setPower(0.0);

        }


}

