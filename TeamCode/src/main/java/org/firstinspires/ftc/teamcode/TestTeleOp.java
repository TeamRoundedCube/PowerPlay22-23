package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp")
public class TestTeleOp extends OpMode {
    float turnPower;
    float forwardPower;
    int maxSpeed = 1;
    AadilHardwareFullBot Robot = new AadilHardwareFullBot();

    @Override
    public void init() {

        Robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
      //  Robot.armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  Robot.armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //@Override
    public void loop() {
        float turnPower = -gamepad1.right_stick_x;
        float forwardPower = -gamepad1.right_stick_y;
        int maxSpeed = 1;

        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            Robot.left.setPower(-turnPower * maxSpeed);
            Robot.right.setPower(turnPower * maxSpeed);
        } else if (Math.abs(gamepad1.right_stick_y) > 0.1) {
            // Forward (Left stick Y)
            Robot.left.setPower(forwardPower * maxSpeed);
            Robot.right.setPower(forwardPower * maxSpeed);
        } else {
            Robot.left.setPower(0);
            Robot.right.setPower(0);
            Robot.armleft.setPower(0);
            Robot.armright.setPower(0);

        }

//        if (Math.abs(gamepad1.right_stick_y) > 0.9 && Math.abs(gamepad1.right_stick_x) > 0.9) {
//            double JSANG = Math.toDegrees(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x));
//            int joystickang = (int)JSANG;
//            Robot.armleft.setTargetPosition(joystickang);
//        }
//        else{
//            Robot.armleft.setPower(0);
//        }
    }

}

