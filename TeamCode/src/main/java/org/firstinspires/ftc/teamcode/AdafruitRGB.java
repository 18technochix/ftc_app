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
        robot.autoInit(hardwareMap, true);

        // get a reference to our DeviceInterfaceModule object.
        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            int[] crgb_blue = robot.color.getCRGB(robot.PORT_BLUE);
            int[] crgb_red = robot.color.getCRGB(robot.PORT_RED);


            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", "r:%d b:%d", crgb_red[0], crgb_blue[0]);
            telemetry.addData("  Red", "r:%d b:%d", crgb_red[1], crgb_blue[1]);
            telemetry.addData("Green", "r:%d b:%d", crgb_red[2], crgb_blue[2]);
            telemetry.addData(" Blue", "r:%d b:%d", crgb_red[3], crgb_blue[3]);
            telemetry.addData("Hue", "r:%3f b:%3f", robot.color.getHue(robot.PORT_RED), robot.color.getHue(robot.PORT_BLUE));

            telemetry.update();
        }
    }
}
