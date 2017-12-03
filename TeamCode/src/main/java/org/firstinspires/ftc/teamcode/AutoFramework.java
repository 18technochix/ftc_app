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
public class AutoFramework extends LinearOpMode{

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


        /*particleLift = hardwareMap.servo.get("particle lift");*/

        //ensure everything is going in the direction you want
        /*robot.fl.setDirection(DcMotor.Direction.REVERSE);
        robot.bl.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/

        //when using encoders, do this, don't ask why (check the yellow postit on the desktop)
      /*leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        //set the initial values for ya servos
        //servos work for values from 0-1, you can use the MR program to determine
        //values from 0-255 and use the corresponding fraction:
      /*particleLift.setPosition(250. / 255.);*/

        //set the power for all of your motors to 0. why? because i said so. don't make robo move just yet


        //Here's the ticket yo, everything before this is initialization, and after this is all of
        //the actual robot-moving stuff
        waitForStart();
        telemetry.addData("Heading", robot.getHeading());
        //THIS IS THE IMPORTANT PART AND I'M TYPING IN CAPS SO THAT YOU NOTICE THIS HOPEFULLY k.
        //In here, you put the actual code. My recommendation is that you write methods to do the bulk
        //of the work, and then insert in between steps
        //e.g. drive off base, then call findTape(), etc
        spinRobot(180, 0.5);
    }

    //METHODS
    public void moveThatRobot(double power, double inches, String direction, double timeout){
        //example shell of a method for y'all to use :) you're welcome
    double wheelCircumference = 2 * (Math.PI) * 2;
       int revolutions = (int) (inches / wheelCircumference);
        int encoderTicks = 1120 * revolutions;
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int position = robot.fr.getCurrentPosition();
        while( position < encoderTicks){
            if(direction == "left"){
                robot.fr.setPower(-power);
                robot.br.setPower(power);
                robot.fl.setPower(power);
                robot.bl.setPower(-power);
            } else if (direction == "right") {
                robot.fr.setPower(-power);
                robot.br.setPower(power);
                robot.fl.setPower(power);
                robot.bl.setPower(-power);
            }else if(direction == "forward"){
                robot.fr.setPower(power);
                robot.br.setPower(power);
                robot.fl.setPower(power);
                robot.bl.setPower(power);
            }else if(direction == "back"){
                robot.fr.setPower(-power);
                robot.br.setPower(-power);
                robot.fl.setPower(-power);
                robot.bl.setPower(-power);
            }
        }




    }

    //knocks jewel off according to color
    public void jewelKnocker(boolean alliance, int color){
        extendBopper();
        color = this.getColor(robot.jewelSensor);
        if(alliance) {
            if (color >= robot.redMin && color <= 360 || color >= 0 && color <= robot.redMax) {
                moveThatRobot(.5, 2, "front", 0);
            } else {
                moveThatRobot(.5, 2, "back", 0);
            }
        } else if(!alliance){
                if (color >= robot.blueMin && color <= robot.blueMax){
                    moveThatRobot(.5, 2, "left", 0);
                } else {
                    moveThatRobot(.5, 2, "right", 0);
                }
        }
        retractBopper();
    }

    public String crytographReader(){
        VuforiaTrackables relicTrackables = robot.picReader.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        /** yeah fam idk what this does but its in the vuforia code and it won't work bc of the pose variable
         * if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
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
        */
        String side = vuMark.toString();
        return side; //yeah also don't know if this works but vuMark is an enum but it's also not an enum its a "RelicRecoveryVuMark" so it's a little confusing
    }
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
    public void findCryptoBox(){

    }

    public void extendBopper(){
        robot.bopperMotor.setTargetPosition(robot.bopperEncoderCount);
        robot.jewelServo.setPosition(180);
    }

    public void retractBopper(){
        robot.bopperMotor.setTargetPosition(-robot.bopperEncoderCount);
        robot.jewelServo.setPosition(0);
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

        } //timeout


}

