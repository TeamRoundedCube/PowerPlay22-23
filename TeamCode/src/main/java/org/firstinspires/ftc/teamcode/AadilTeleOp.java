package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "AadilTeleOp")
public class AadilTeleOp extends OpMode {
    float turnPower;
    float forwardPower;
    float armpower;
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

    }

    //@Override
    public void loop() {

        turnPower = -gamepad1.right_stick_x;
        forwardPower = -gamepad1.right_stick_y;
        armpower = -gamepad1.left_stick_y;

        telemetry.addData("Armleft", Robot.armleft.getCurrentPosition());
        telemetry.addData("armright", Robot.armright.getCurrentPosition());
        telemetry.update();

        if (Math.abs(gamepad1.right_stick_x) > 0.1) {

            Robot.left.setPower(-turnPower * maxSpeed);
            Robot.right.setPower(turnPower * maxSpeed);
        }
        else{
            Robot.left.setPower(0);
            Robot.right.setPower(0);
        }
        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            // Forward (Left stick Y)
            Robot.armleft.setPower(armpower * maxSpeed);
             Robot.armright.setPower(armpower * maxSpeed);

        }
        else{
            Robot.armright.setPower(0);
            Robot.armleft.setPower(0);
        }
        if (Math.abs(gamepad1.right_stick_y) > 0.1) {
            // Forward (Left stick Y)
            Robot.left.setPower(forwardPower * maxSpeed);
            Robot.right.setPower(forwardPower * maxSpeed);
        }
        else{
            Robot.left.setPower(0);
            Robot.right.setPower(0);
        }
//
//        int timespressed = 0;
//        if (gamepad1.a){
//            Robot.servo.setPosition(0);
//        }
//        if (gamepad1.b){
//            Robot.servo.setPosition(0.2);
//        }

    }
    public void moveArm(double speed, int position) {
        Robot.armleft.setPower(speed);
        Robot.armright.setPower(speed);
        Robot.armleft.setTargetPosition(position);
        Robot.armright.setTargetPosition(position);
       // if (Robot.armleft.getCurrentPosition() != Robot.armleft.getTargetPosition()) {
       //     Robot.armleft.setPower(speed);
         //   Robot.armright.setPower(speed);
           // Robot.armleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Robot.armright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // }
        //Robot.armleft.setPower(0);
        //Robot.armright.setPower(0);
        /// Kyran changed on 1-2-2022 to try to make robot fast again
        /*while(robot.arm.isBusy()) {
            sleep(1);
        }
        robot.arm.setPower(0);*/
    }

    }

