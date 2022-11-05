//  ____          ____  ____           ____   ______________                     ______            ____        ____
// |    |       /    /  \   \        /    /  |     _______   \                 /        \         |     \     |   |
// |    |     /    /     \   \     /    /    |   |        |   \              /    / \    \        |      \    |   |
// |    |   /    /        \   \  /    /      |   |_______|    /            /    /    \    \       |       \   |   |
// |    | /    /           \   \/   /        |     ___      _/           /    /       \    \      |    |\  \  |   |
// |    | \    \            |     |          |    |   \    \           /    /__________\    \     |    | \  \ |   |
// |    |  \    \           |     |          |    |    \    \        /    /_____________\    \    |    |  \       |
// |    |   \    \          |     |          |    |     \    \     /    /                \    \   |    |   \      |
// |____|    \____\         |_____|          |____|      \____\  /____/                   \____\  |____|    \_____|

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

//Created by Kyran 10/26/2022 @ 4:51pm
//Purpose: Testing a working drive bot

@TeleOp(name = "NovTeleop")
public class KyranTeleop extends OpMode {

    AadilHardwareFullBot robot = new AadilHardwareFullBot();
    float turnPower; //Turn robot
    float forwardPower; //FOrward and Back
    float maxLeftSpeed = 1;
    double maxRightSpeed = 0.8;
   // boolean shooting = false;
   // boolean autoAim = false;
   // boolean squared = false;
   // boolean light = false;
   // boolean tooClose = false;
    // Code to run ONCE when the driver hits INIT

    @Override
    public void init() {


        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        robot.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
      //  robot.armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  robot.armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {

        //telemetry.addData("Left Encoder Position", robot.left.getCurrentPosition());
        //telemetry.addData("Right Encoder Position", robot.right.getCurrentPosition());

        telemetry.addData("Armleft", robot.armleft.getCurrentPosition());
        telemetry.addData("armright", robot.armright.getCurrentPosition());
        telemetry.update();
        //Variables

        turnPower = -gamepad1.right_stick_x; //Turn robot
        forwardPower = -gamepad1.left_stick_y; //FOrward and Back
        //Gamepad1

        if (Math.abs(gamepad1.right_stick_x) > 0.1 ) {

            robot.left.setPower(-turnPower * maxLeftSpeed);
            robot.right.setPower(turnPower * maxRightSpeed);
        }
        else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            // Forward (Left stick Y)
            robot.left.setPower(forwardPower * maxLeftSpeed);
            robot.right.setPower(forwardPower * maxRightSpeed);
//        } else if (gamepad1.a) {
//            robot.armleft.setPower(1);
//            robot.armright.setPower(1);
        }
        else if (gamepad2.right_trigger > 0.1){
            robot.servo.setPosition(0);
        }
        else if (gamepad2.left_trigger > 0.1){
            robot.servo.setPosition(0.5);
        }
        else {
            robot.left.setPower(0);
            robot.right.setPower(0);
        }
        //Gamepad2
if (gamepad2.a) {
    moveArm(0.8, 108, 86);
} else if (gamepad2.b) {
    moveArm(0.8, 324, 310);
} else if (gamepad2.x) {
    moveArm(0.8, 182, 182);
        }
//}

        // Arm Position Values

// left 108, right 86 (low)
// left (324), right 310 (highest)
        // left 163, right 162 (med)
        int downPosition = 0;
        int drivingPosition = 350;
        int levelOne = 3000;
        int levelTwo = 2600;
        int levelThree = 2000;
        int elementPos = 3500;
        int elementDrop = 1800;
        double armSpeed = 0.75;

        //Arm
        if (gamepad2.dpad_down) { //Intake position
            moveArm(0.5, 0, 0);
        }/* else if (gamepad2.dpad_up) { // Level 1
            // robot.basket.setPosition(0.45);
            moveArm(armSpeed, levelOne);
        } else if (gamepad2.dpad_left) { //Level 2
            //  robot.basket.setPosition(0.45);
            moveArm(armSpeed, levelTwo);
        } else if (gamepad2.dpad_up) { //Level 3
            //  robot.basket.setPosition(0.45);
            moveArm(armSpeed, levelThree);
        }

        if (gamepad2.y) {
            if ((robot.front_distance.getDistance(DistanceUnit.CM) <= 16))
            //|| (robot.left_distance.getDistance(DistanceUnit.CM) <= 40))
            //|| (robot.right_fr_distance.getDistance(DistanceUnit.CM) <= 40))
            {
                robot.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                tooClose = true;
                telemetry.addData("front_distance: ", robot.front_distance.getDistance(DistanceUnit.CM));
                // telemetry.addData("left_distance: ", robot.left_distance.getDistance(DistanceUnit.CM));
                // telemetry.addData("right_fr_distance: ", robot.right_fr_distance.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            else if ((robot.front_distance.getDistance(DistanceUnit.CM) > 16) &&(robot.front_distance.getDistance(DistanceUnit.CM) <= 21) )
            {
                robot.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                telemetry.addData("front_distance: ", robot.front_distance.getDistance(DistanceUnit.CM));
                //telemetry.addData("left_distance: ", robot.left_distance.getDistance(DistanceUnit.CM));
                // telemetry.addData("right_fr_distance: ", robot.right_fr_distance.getDistance(DistanceUnit.CM));
            }
            else
            {
                robot.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                telemetry.addData("front_distance: ", robot.front_distance.getDistance(DistanceUnit.CM));
            }
        }

        //Basket: 1 - 0.3   (Open: 0.45)
        if (gamepad2.x) {
            robot.basket.setPosition(1);
            sleep(1000);
            if(robot.arm.getCurrentPosition() != downPosition) {
                moveArm(armSpeed, downPosition);
                robot.basket.setPosition(0.3);
                sleep(300);
            }
        }
// Element: 0.95 - 0.45     (0.6 Pickup Position) (0.95 Default)
/*        if(gamepad2.b) {
            robot.element.setPosition(0.6);
        }
        if(gamepad2.a) {
            robot.element.setPosition(0.95);
        }
*/
/*
        //X button Override
        if (gamepad2.x) {
            robot.arm.setPower(0);
            robot.basket.setPosition(0.35);
            sleep(1000);
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.claw.resetDeviceConfigurationForOpMode();
        }
*/
        //telemetry.addData("Distance (cm)",
        //      String.format(Locale.US, "%.02f", robot.color_left.getDistance(DistanceUnit.CM)));
       /* telemetry.addData("Alpha", robot.color_left.alpha());
        telemetry.addData("Red  ", robot.color_left.red());
        telemetry.addData("Green", robot.color_left.green());
        telemetry.addData("Blue ", robot.color_left.blue());
        */
        //telemetry.addData("Hue", hsvValues[0]);

/*
        telemetry.addData("Encoder", robot.arm.getCurrentPosition());
        telemetry.addData("Basket", robot.basket.getPosition());
        telemetry.addData("LeftTrigger", gamepad1.left_trigger);
        telemetry.addData("Left Distance (cm): ", robot.left_distance.getDistance(DistanceUnit.CM));
        telemetry.update();
*/
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.armleft.setPower(0);
        robot.armright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
/*
    public void moveArm(double speed, int leftPosition, int rightPosition) {
        if (robot.armleft.getCurrentPosition() > leftPosition && robot.armright.getCurrentPosition() > rightPosition) {
            robot.armleft.setPower(-speed);
            robot.armright.setPower(-speed);
        } else if (robot.armleft.getCurrentPosition() < leftPosition && robot.armright.getCurrentPosition() < rightPosition) {
            robot.armleft.setPower(speed);
            robot.armright.setPower(speed);
        } else {
            robot.armleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armleft.setPower(0);
            robot.armright.setPower(0);
        }
    }
*/
    public void moveArm(double speed, int leftPosition, int rightPosition) {
        robot.armleft.setPower(speed);
        robot.armright.setPower(speed);
        robot.armleft.setTargetPosition(leftPosition);
        robot.armright.setTargetPosition(rightPosition);
        //robot.armleft.getCurrentPosition();
        robot.armright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        /// Kyran changed on 1-2-2022 to try to make robot fast again
        /*while(robot.arm.isBusy()) {
            sleep(1);
        }
        robot.arm.setPower(0);*/
    }
/*
    public void testArm() {
        robot.left.setTargetPosition(3000);
        telemetry.addData("position", "position");
        telemetry.update();
        robot.arm.setPower(0.5);
        telemetry.addData("power", "power");
        telemetry.update();
        // Switch to RUN_TO_POSITION mode
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("run", "position");
        telemetry.update();

        // Start the motor moving by setting the max velocity to 200 ticks per second


        // While the Op Mode is running, show the motor's status via telemetry
        /*while (robot.arm.isBusy()) {
            telemetry.addData("is", "busy");
            telemetry.update();
           sleep(1);
        }*/
//        telemetry.addData("not", "busy");
//        telemetry.update();
//    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

//}
}
