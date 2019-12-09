package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.channels.SeekableByteChannel;

public class ArmBot {
    private DcMotor armMotor, leftMotor, rightMotor;
    Servo handServo1, handServo2, plateArm;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode opmode;

    double DRIVE_COUNTS_PER_REV = 288;
    double DRIVE_WHEEL_DIAMETER_MM = 90;
    double DRIVE_WHEEL_DIAMETER_CM = DRIVE_WHEEL_DIAMETER_MM / 10;
    double DRIVE_WHEEL_DIAMETER_INCHES = DRIVE_WHEEL_DIAMETER_CM / 2.54;
    double DRIVE_COUNTS_PER_INCH = DRIVE_COUNTS_PER_REV / (DRIVE_WHEEL_DIAMETER_INCHES * Math.PI);

    double ARM_COUNTS_PER_SHAFT_REV = 1120;
    double ARM_S1G1_TEETH = 60;
    double ARM_S1G2_TEETH = 90;
    double ARM_S1_RATIO = ARM_S1G1_TEETH / ARM_S1G2_TEETH;
    double ARM_S2G1_TEETH = 30;
    double ARM_S2G2_TEETH = 125;
    double ARM_S2_RATIO = ARM_S2G1_TEETH / ARM_S2G2_TEETH;
    double ARM_GEAR_RATIO = ARM_S1_RATIO * ARM_S2_RATIO;
    double ARM_COUNTS_PER_GEAR_REV = ARM_COUNTS_PER_SHAFT_REV / ARM_GEAR_RATIO;
    double ARM_COUNTS_PER_DEGREE = ARM_COUNTS_PER_GEAR_REV / 360;


    public void init(HardwareMap hwMap, Telemetry tel, LinearOpMode mode) {
        hardwareMap = hwMap;
        telemetry = tel;
        opmode = mode;

        armMotor = hardwareMap.get(DcMotor.class, "arm");
        leftMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        handServo1 = hardwareMap.get(Servo.class, "hand1");
        handServo2 = hardwareMap.get(Servo.class, "hand2");
        plateArm = hardwareMap.get(Servo.class, "plate");

        handServo1.setDirection(Servo.Direction.REVERSE);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        int newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * DRIVE_COUNTS_PER_INCH);
        int newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * DRIVE_COUNTS_PER_INCH);

        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        leftMotor.setPower(Math.abs(speed));
        rightMotor.setPower(Math.abs(speed));

        while(runtime.seconds() < timeout && leftMotor.isBusy() && rightMotor.isBusy()) {
//            telemetry.addData("LeftProgress", "%7/%7", leftMotor.getCurrentPosition(), newLeftTarget);
//            telemetry.addData("RightProgress", "%7/%7", rightMotor.getCurrentPosition(), newRightTarget);
            telemetry.addData("Status", "Driving");
            telemetry.update();
        }

        driveStop();
    }

    public void driveStop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    public int getArmPosition() { return armMotor.getCurrentPosition(); }

    public void armAddAngle(double angle) {
        int newTarget = armMotor.getCurrentPosition() + (int)(angle * ARM_COUNTS_PER_DEGREE);

        armMotor.setTargetPosition(newTarget);

        opmode.sleep((long)(10.0 * angle));
    }

    public void grab() {
        handServo1.setPosition(0.0);
        handServo2.setPosition(0.4);
//        opmode.sleep(500);
    }

    public void letgo() {
        handServo1.setPosition(0.5);
        handServo2.setPosition(0.55);
//        opmode.sleep(500);
    }

    public void plateArmDown() {
        plateArm.setPosition(1.0);
    }

    public void plateArmUp() {
        plateArm.setPosition(0.5);
    }
}
