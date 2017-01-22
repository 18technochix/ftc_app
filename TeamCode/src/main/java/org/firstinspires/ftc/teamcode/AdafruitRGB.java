package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Techno Team_PC_III on 12/31/2016.
 */
//@Disabled
@Autonomous(name = "Sensorrrrrrr: AdafruitRGB", group = "Autonomous")
public class AdafruitRGB extends LinearOpMode {
    GoldilocksHardware robot           = new GoldilocksHardware(this);

    public void runOpMode(){
        robot.teleInit(hardwareMap);
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to our DeviceInterfaceModule object.
        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {

            // convert the RGB values to HSV values.
            Color.RGBToHSV((robot.colorBlue.red() * 255) / 800, (robot.colorBlue.green() * 255) / 800, (robot.colorBlue.blue() * 255) / 800, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", robot.colorBlue.alpha());
            telemetry.addData("Red  ", robot.colorBlue.red());
            telemetry.addData("Green", robot.colorBlue.green());
            telemetry.addData("Blue ", robot.colorBlue.blue());
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.update();
        }
    }
}
