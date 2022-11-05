package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class NovAutoEncoder extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    AadilHardwareFullBot robot = new AadilHardwareFullBot();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        waitForStart();

        if (opModeIsActive()) {
            driveBack(1,-1000);
            sleep(100);
            turnRight(.5, 250);
            sleep(100);
            moveArm(0.8, 324, 310);
            driveStraight(.9, 400);
            sleep(100);
            driveBack(.5, 400);
            turnLeft(.5, 200);
        }

    }

    public void driveStraight(double speed, double distance){
        double maxLeftSpeed = 0.9;
        double maxRightSpeed = 0.8;
        robot.left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (opModeIsActive()) {

            double i = robot.right.getCurrentPosition();
            while (opModeIsActive() && i < distance) {
                robot.left.setPower(-(maxLeftSpeed * speed));
                robot.right.setPower(-(maxRightSpeed * speed));
                i = robot.right.getCurrentPosition();
            }

            robot.left.setPower(0);
            robot.right.setPower(0);

        }
    }


    /**
     * Moves the robot backwards by an amount based on the given parameters speed and seconds
     * @param speed
     * @param distance
     */
    public void driveBack(double speed, double distance){
        robot.left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double maxLeftSpeed = 0.9;
        double maxRightSpeed = 0.8;
        robot.right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (opModeIsActive()) {

            double i = robot.right.getCurrentPosition();
            //double j = robot.left.getCurrentPosition();

            while (opModeIsActive() && i > distance) {//(i < distance || j < distance) ) {
//if(j<distance) {
  //  robot.right.setPower(-(maxRightSpeed * speed));
//}
//if(i>distance) {
    robot.left.setPower(-(maxLeftSpeed * speed));
    robot.right.setPower(-(maxRightSpeed * speed));
//}
                i = robot.right.getCurrentPosition();
                //j = robot.right.getCurrentPosition();

            }

            robot.left.setPower(0);
            robot.right.setPower(0);

        }
    }

    /**
     * Turns the robot left based on the given parameters speed and seconds.
     * @param speed
     * @param distance
     */
    public void turnLeft(double speed, double distance){
        double maxLeftSpeed = 0.9;
        double maxRightSpeed = 0.8;
        robot.left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (opModeIsActive()) {

            double i = robot.right.getCurrentPosition();

            while (opModeIsActive() && i < distance) {
                robot.left.setPower((maxLeftSpeed * speed));
                robot.right.setPower(-(maxRightSpeed * speed));
//}
                i = robot.right.getCurrentPosition();
                //j = robot.right.getCurrentPosition();

            }

            robot.left.setPower(0);
            robot.right.setPower(0);

        }
    }


    /**
     * Turns the robot right based on the given parameters speed and seconds.
     * @param speed
     * @param distance
     */
    public void turnRight(double speed, double distance){
        double maxLeftSpeed = 0.9;
        double maxRightSpeed = 0.8;
        robot.left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (opModeIsActive()) {

            double i = robot.right.getCurrentPosition();
            //double j = robot.left.getCurrentPosition();

            while (opModeIsActive() && i > distance) {//(i < distance || j < distance) ) {
//if(j<distance) {
                //  robot.right.setPower(-(maxRightSpeed * speed));
//}
//if(i>distance) {
                robot.left.setPower(-(maxLeftSpeed * speed));
                robot.right.setPower((maxRightSpeed * speed));
//}
                i = robot.right.getCurrentPosition();
                //j = robot.right.getCurrentPosition();

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
