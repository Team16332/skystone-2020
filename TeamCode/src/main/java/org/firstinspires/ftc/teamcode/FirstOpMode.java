package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class FirstOpMode extends LinearOpMode {
    private DcMotor armMotor, leftMotor, rightMotor;
    private Servo handServo1, handServo2;

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        handServo1 = hardwareMap.get(Servo.class, "hand1");
        handServo2 = hardwareMap.get(Servo.class, "hand2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        while(opModeIsActive()) {
//            armMotor.setPower(-this.gamepad1.left_stick_y);
            leftMotor.setPower(-this.gamepad1.left_stick_y);
            rightMotor.setPower(-this.gamepad1.right_stick_y);

            telemetry.addData("MotorPower", armMotor.getPower());
            telemetry.addData("LeftPower", leftMotor.getPower());
            telemetry.addData("RightPower", rightMotor.getPower());
            telemetry.update();

            if (this.gamepad1.left_trigger > 0 && this.gamepad1.right_trigger > 0) {
                armMotor.setPower(0);
            }
            else if (this.gamepad1.right_trigger > 0) {
                armMotor.setPower(1);
            } else if (this.gamepad1.left_trigger > 0) {
                armMotor.setPower(-1);
            } else {

            }
        }
    }
}
