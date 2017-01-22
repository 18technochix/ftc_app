package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Techno Team_PC_III on 1/6/2017.
 */
@Autonomous (name="AutoShoot", group="Autonomous")
public class AutoShoot extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    /* Declare OpMode members. */
    GoldilocksHardware robot   = new GoldilocksHardware(this);

    public void runOpMode() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.autoInit(hardwareMap);

        waitForStart();

        robot.runShooter(.45);
    }
}
