package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "AadilTeleOp")
public class AadilTeleOp extends OpMode {

    AadilHardwareFullBot Robot = new AadilHardwareFullBot();

    @Override
    public void init() {

        Robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void init_loop() {
        Robot.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void start() {
    }


    @Override
    public void loop() {

        float turnPower = -gamepad1.right_stick_x;
        float forwardPower = -gamepad1.left_stick_y;
        float strafePower = gamepad1.left_stick_x;

        int maxSpeed = 1;
        if (Math.abs(gamepad1.right_stick_x) > 0.1) {

            Robot.front_left.setPower(-turnPower * maxSpeed);
            Robot.front_right.setPower(turnPower * maxSpeed);
            Robot.back_left.setPower(-turnPower * maxSpeed);
            Robot.back_right.setPower(turnPower * maxSpeed);
        } else {
            Robot.front_left.setPower(0);
            Robot.front_right.setPower(0);
            Robot.back_left.setPower(0);
            Robot.back_right.setPower(0);
        }

        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            // Forward (Left stick Y)
            Robot.front_left.setPower(forwardPower * maxSpeed);
            Robot.front_right.setPower(forwardPower * maxSpeed);
            Robot.back_left.setPower(forwardPower * maxSpeed);
            Robot.back_right.setPower(forwardPower * maxSpeed);
        } else {
            Robot.front_left.setPower(0);
            Robot.front_right.setPower(0);
            Robot.back_left.setPower(0);
            Robot.back_right.setPower(0);
        }
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            // Strafe (Left stick X)
            Robot.front_left.setPower(strafePower * maxSpeed);
            Robot.front_right.setPower(-strafePower * maxSpeed);
            Robot.back_left.setPower(-strafePower * maxSpeed);
            Robot.back_right.setPower(strafePower * maxSpeed);
        } else {
            Robot.front_left.setPower(0);
            Robot.front_right.setPower(0);
            Robot.back_left.setPower(0);
            Robot.back_right.setPower(0);
        }
    }
}