package org.firstinspires.ftc.teamcode;

/**
 * Created by Techno Team_PC_III on 8/10/2017.
 */

import com.qualcomm.ftccommon.configuration.ScannedDevices;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test TeleOp", group="Linear Opmode")
public class TestTeleOp extends LinearOpMode {
//Variables

    public DcMotor collector=null;
    public DcMotor driveLeft=null;
    public DcMotor driveRight=null;
    public DcMotor gathererArm=null;
    public DcMotor gathererSpinner=null;
    public DcMotor buttonBopper=null;
    public DcMotor shooter=null;

    public Servo particleLift=null;

    //Constants
    public static final double particleLiftUp=(175./255.);
    public static final double particleLiftDown = (250./255.);
    public static final int maxBop = 3680;
    double p=0.;


    @Override
    public void runOpMode() throws InterruptedException {
//Configurations
        collector = hardwareMap.dcMotor.get("collector");
        driveLeft = hardwareMap.dcMotor.get("lDrive");
        driveRight = hardwareMap.dcMotor.get("rDrive");
        gathererArm = hardwareMap.dcMotor.get("arm");
        gathererSpinner = hardwareMap.dcMotor.get("spinner");
        buttonBopper = hardwareMap.dcMotor.get("bBopper");
        shooter = hardwareMap.dcMotor.get("shooter");

        particleLift = hardwareMap.servo.get("lift");
        particleLift.setPosition(particleLiftDown);

        driveRight.setDirection(DcMotorSimple.Direction.FORWARD);
        driveLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        buttonBopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //RUN_USING_ENCODER
        //buttonBopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        collector.setPower(0.);
        driveLeft.setPower(0.);
        driveRight.setPower(0.);
        gathererArm.setPower(0.);
        gathererSpinner.setPower(0.);
        buttonBopper.setPower(0.);
        shooter.setPower(0.);

        waitForStart();

        while(opModeIsActive()){

            //drive
            double pR = (-gamepad1.right_stick_y);
            double pL = (-gamepad1.left_stick_y);
            driveLeft.setPower(pL);
            driveRight.setPower(pR);

            //particle lift
            if(gamepad2.dpad_up){
                particleLift.setPosition(particleLiftUp);
            }
            else{
                particleLift.setPosition(particleLiftDown);
            }

            //Button Bopper
            if(gamepad1.dpad_left && (buttonBopper.getCurrentPosition() < maxBop)){
                buttonBopper.setPower(.92);
            }
            else if(gamepad1.dpad_right && (buttonBopper.getCurrentPosition() > -maxBop)){
                buttonBopper.setPower(-.923);
            }
            else{
                buttonBopper.setPower(0.0);
            }

            //collector
            collector.setPower((gamepad2.right_trigger - gamepad2.left_trigger));

            //gatherer
            if(gamepad2.dpad_left){
                gathererArm.setPower(.2);
            }
            else if(gamepad2.dpad_right){
                gathererArm.setPower(-.2);
            }
            else{
                gathererArm.setPower(0.);
            }

            if(gamepad1.left_bumper){
                gathererSpinner.setPower(.927);
            }
            else if(gamepad1.right_bumper){
                gathererSpinner.setPower(-.926);
            }
            else{
                gathererSpinner.setPower(0.);
            }

            //Shooter(duhn duhn DDDUUUUUHHHNNNNN)
            double motorIncrement= .01;

            if (gamepad2.y){                    //12
                if (p < .7) {
                    p = p + motorIncrement;
                }
                else if (p >= .7) {
                    p = .7;
                }
            }
            else if (gamepad2.b){               //3
                if (p < .4) {
                    p = p + motorIncrement;
                }
                else if (p >= .4) {
                    p = .4;
                }
            }
            else if (gamepad2.a){               //6
                if (p < .5) {
                    p = p + motorIncrement;
                }
                else if (p >= .5) {
                    p = .5;
                }
            }
            else if (gamepad2.x){               //9
                if (p < .6) {
                    p = p + motorIncrement;
                }
                else if (p >= .6) {
                    p = .6;
                }
            }
            else{                               //slowly decrease power once button released
                if (p > 0.){
                    p = p - motorIncrement;
                    if (p < 0.){
                        p = 0.;
                    }
                }
                else{ //p <= 0 ?
                    p = 0.;
                }
            }

            shooter.setPower(p);          //set shooter power

        }

    }
}
