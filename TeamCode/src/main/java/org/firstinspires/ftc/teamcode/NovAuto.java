package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class NovAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    AadilHardwareFullBot robot = new AadilHardwareFullBot();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();

        if (opModeIsActive()) {
            driveBack(1,39);
            sleep(100);
            turnRight(.5, 49);
            sleep(100);
            moveArm(0.8, 324, 310);
            driveStraight(.9, 21);
            sleep(100);
            driveBack(.5, 21);
            turnLeft(.5, 30);
        }

    }

    public void driveStraight(double speed, double seconds){
        double maxLeftSpeed = 0.9;
        double maxRightSpeed = 0.8;
        robot.left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds * 1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds) {

                robot.right.setPower(maxRightSpeed*speed);
                robot.left.setPower(maxLeftSpeed*speed);

                i++;
            }

            robot.left.setPower(0);
            robot.right.setPower(0);

        }
    }


    /**
     * Moves the robot backwards by an amount based on the given parameters speed and seconds
     * @param speed
     * @param seconds
     */
    public void driveBack(double speed, double seconds){
        robot.left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        double maxLeftSpeed = 0.9;
        double maxRightSpeed = 0.8;
        robot.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds * 1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds) {

                robot.right.setPower(-(maxRightSpeed*speed));
                robot.left.setPower(-(maxLeftSpeed*speed));

                i++;
            }

            robot.left.setPower(0);
            robot.right.setPower(0);

        }
    }

    /**
     * Turns the robot left based on the given parameters speed and seconds.
     * @param speed
     * @param seconds
     */
    public void turnLeft(double speed, double seconds){
        double maxLeftSpeed = 0.9;
        double maxRightSpeed = 0.8;
        robot.left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds * 1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds) {

                robot.right.setPower(-(maxRightSpeed*speed));
                robot.left.setPower((maxLeftSpeed*speed));

                i++;
            }

            robot.left.setPower(0);
            robot.right.setPower(0);

        }
    }


    /**
     * Turns the robot right based on the given parameters speed and seconds.
     * @param speed
     * @param seconds
     */
    public void turnRight(double speed, double seconds){
        double maxLeftSpeed = 0.9;
        double maxRightSpeed = 0.8;
        robot.left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds * 1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds) {

                robot.right.setPower(maxRightSpeed*speed);
                robot.left.setPower(-(maxLeftSpeed*speed));

                i++;
            }

            robot.left.setPower(0);
            robot.right.setPower(0);

        }
    }
    public void moveArm(double speed, int leftPosition, int rightPosition) {
        robot.armleft.setPower(speed);
        robot.armright.setPower(speed);
        robot.armleft.setTargetPosition(leftPosition);
        robot.armright.setTargetPosition(rightPosition);
        //robot.armleft.getCurrentPosition();
        robot.armleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.armleft.isBusy() || robot.armright.isBusy()) {
            sleep(1);
        }
        robot.armleft.setPower(0);
        robot.armright.setPower(0);


        /// Kyran changed on 1-2-2022 to try to make robot fast again
        /*while(robot.arm.isBusy()) {
            sleep(1);
        }
        robot.arm.setPower(0);*/
    }
}
