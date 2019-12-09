package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Test: Encoders",group = "Encoder")
public class AutoTest extends LinearOpMode {
    ArmBot robot = new ArmBot();

    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 0.8;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            robot.encoderDrive(DRIVE_SPEED, -22, -22, 22); // Drive 12 inches forward, timeout in 2 secs

//            robot.armAddAngle(180); // Rotate the arm 180 degrees
//            robot.grab(); // Grab the stone
//            robot.armAddAngle(-30); // Lift up the stone

//            robot.encoderDrive(TURN_SPEED, -4, 4, 4); // Turn left, wheels move 4 inches, times out in 4 secs
//            robot.encoderDrive(DRIVE_SPEED, 2, 2, 1); // Drive 2 inches forward, timeout in 1 secs
        }
    }
}
