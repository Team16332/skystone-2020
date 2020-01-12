package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Move Build Plate")
public class AutoPlate extends LinearOpMode {
    ArmBot robot = new ArmBot();

    static final double DRIVE_SPEED = 1.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            robot.encoderDrive(DRIVE_SPEED, -69, -69, 22);
        }
    }
}
