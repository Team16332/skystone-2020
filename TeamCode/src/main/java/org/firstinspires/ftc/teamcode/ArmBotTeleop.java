package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Default TeleOp for ArmBot",group="ArmBot")
public class ArmBotTeleop extends LinearOpMode {
    ArmBot robot = new ArmBot();

    static final double ARM_SPEED = 0.75;
    static final double ARM_STEP = 0.2;

    static final double SPEED = 0.8;

    double armAngle = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap,telemetry,this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while(opModeIsActive()) {
            float vertical = this.gamepad1.left_stick_y * (float) SPEED;
            float horizontal = this.gamepad1.right_stick_x * (float) SPEED;
            float leftSpeed = vertical - horizontal;
            float rightSpeed = vertical + horizontal;
            robot.tankDrive(leftSpeed, rightSpeed);

            if (this.gamepad1.right_trigger > 0) {
//                armAngle += ARM_STEP;
                robot.setArmPower(ARM_SPEED);
            } else if (this.gamepad1.left_trigger > 0) {
//                armAngle -= ARM_STEP;
                robot.setArmPower(-ARM_SPEED);
            } else {
                robot.setArmPower(0);
            }

//            int currentPosition = robot.getArmPosition();
//            int targetPosition = (int)(armAngle * robot.ARM_COUNTS_PER_DEGREE);
//            int positionDifference = targetPosition - currentPosition;
//            double rawPower = positionDifference / 1120;
//            double clampedPower = Math.max(-1.0, Math.min(1.0, rawPower));
//            robot.setArmPower(clampedPower);

            if (this.gamepad1.right_bumper) {
                robot.letgo();
                telemetry.addData("HandState", "Letgo");
            } else {
                robot.grab();
                telemetry.addData("HandState", "Grab");
            }

            if (this.gamepad1.left_bumper) {
                robot.plateArmDown();
                telemetry.addData("PlateState", "Down");
            } else {
                robot.plateArmUp();
                telemetry.addData("PlateState", "Up");
            }
            telemetry.update();
        }
    }
}
