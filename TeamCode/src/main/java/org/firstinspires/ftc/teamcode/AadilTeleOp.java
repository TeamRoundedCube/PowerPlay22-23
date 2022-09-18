package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "AadilTeleOp", group = "TeleOp")
public class AadilTeleOp extends OpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    @Override
    public void init() {
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        motorFrontLeft.setPower(gamepad1.left_stick_y);
        motorBackLeft.setPower(gamepad1.left_stick_y);
        motorFrontRight.setPower(gamepad1.left_stick_y);
        motorBackRight.setPower(gamepad1.left_stick_y);


    }
}
