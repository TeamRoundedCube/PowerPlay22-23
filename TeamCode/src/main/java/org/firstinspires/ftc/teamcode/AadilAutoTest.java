package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;



@Autonomous
public class AadilAutoTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 400;    // 537 (Original)
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.54;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double LEFT_COEFF = 0.5;
    static final double RIGHT_COEFF = 1.0;

    int position = 2;
    double inches = 18;
    double circum = 3.14*3.54;
    double rotationsNeeded = inches/circum;
    int encoderDrivingTarget = (int) (rotationsNeeded*1120);
    AadilHardwareFullBot robot = new AadilHardwareFullBot();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aadilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aadilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aadilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(960,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aadilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    tagToTelemetry(tagOfInterest);
                } else {

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nwe HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {

                if (tagOfInterest == null) {
                } else {
                    telemetry.addLine("\nwe HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }


            telemetry.update();
            sleep(20);
            robot.servo.setPosition(0.5);
        }





        waitForStart();


        if (opModeIsActive()) {


            driveStraight(0.2, 10000);
//            //Drive and Drop Cone
//            //moveEncoder(0.8, -encoderDrivingTarget);
//            // driveReverse(0.8, 80);
//            driveBack(1,880);
//            sleep(500);
//            switch (tagOfInterest.id) {
//                case 1: turnRight(.5, 380); driveBack(0.7, 950); break;
//                case 3: turnLeft(.5, 380); driveBack(0.6, 990); break;
//                default: robot.left.setPower(0); robot.right.setPower(0); break;
//            }
//            moveArm(0.5, 0, 0);


        }

    }
    public void moveEncoder(double speed, int target) {
        double maxLeftSpeed = 0.4;
        double maxRightSpeed = 1;

        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left.setTargetPosition(target);
        robot.right.setTargetPosition(target);
        robot.left.setPower((maxLeftSpeed * speed));
        robot.right.setPower((maxRightSpeed * speed));
        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive() && robot.left.isBusy() && robot.right.isBusy()){}
        robot.left.setPower(0);
        robot.right.setPower(0);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.left.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.right.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.left.setTargetPosition(newLeftTarget);
            robot.right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.

            runtime.reset();
            robot.right.setPower(Math.abs(speed));
            robot.left.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() && (robot.left.getCurrentPosition() < newLeftTarget) && (robot.right.getCurrentPosition() < newRightTarget) &&
                    (runtime.seconds() < timeoutS))

         /*                   while (opModeIsActive()
                                    && robot.back_left.isBusy()
                                    && robot.back_right.isBusy()
                                    && robot.front_right.isBusy()
                                    && robot.front_left.isBusy()
                                    &&  runtime.seconds() < timeoutS)*/
            {
                //Provides current position and updates it every time it changes
                telemetry.addData("Curr Velocity at time ", "backleft(%.2f), " +
                                "backright (%.2f)",
                        robot.left.getVelocity(),
                        robot.right.getVelocity());
                //sleep(250);
                telemetry.update();
                idle();
            }

            // Stop all motion;
            robot.left.setPower(0);
            robot.right.setPower(0);
        }
    }

    public void encoderDriveReverse(double speed,
                                    double leftInches, double rightInches,
                                    double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        telemetry.addData("DriveReverse", "Running at %7d :%7d  ",
                robot.left.getCurrentPosition(),
                robot.right.getCurrentPosition());
        telemetry.update();
        // Determine new target position, and pass to motor controller
        newLeftTarget = robot.left.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = robot.right.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);

        robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.left.setTargetPosition(newLeftTarget);
        robot.right.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();



        while (opModeIsActive()
                && (robot.left.getCurrentPosition() > newLeftTarget)
                && (robot.right.getCurrentPosition() > newRightTarget)
                && (runtime.seconds() < timeoutS)) {

            robot.right.setPower(-speed * RIGHT_COEFF);
            robot.left.setPower(-speed * LEFT_COEFF);

        }


        robot.left.setPower(0);    // Stop all motion;
        robot.right.setPower(0);

    }
    public void driveReverse (double speed, double distance) {
        encoderDriveReverse(speed, distance, distance, 30);

    }

    public void driveForward(double speed, double distance) {
        encoderDrive(speed, distance, distance, 30);
    }

    public void driveStraight(double speed, double distance){
        double maxLeftSpeed = 0.5;
        double maxRightSpeed = 1;

        // robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {
            double i = robot.right.getCurrentPosition();
            double j = robot.left.getCurrentPosition();
            double leftTarget = j + distance;
            double rightTarget = i + distance;



            while (opModeIsActive() && i < rightTarget && j < leftTarget) {
                telemetry.addData("Left Encoder Position", robot.left.getCurrentPosition());
                telemetry.addData("Right Encoder Position", robot.right.getCurrentPosition());
                telemetry.update();
                if(i < rightTarget){
                    robot.right.setPower((maxRightSpeed * speed));
                    i = robot.right.getCurrentPosition();

                }
                if(j < leftTarget) {
                    robot.left.setPower((maxLeftSpeed * speed));
                    j = robot.left.getCurrentPosition();

                }


            }

            robot.left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.left.setPower(-1);
            robot.right.setPower(-1);
            sleep(50);
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
        double maxLeftSpeed = 0.5;
        double maxRightSpeed = 1;

        // robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {
            double i = robot.right.getCurrentPosition();
            double j = robot.left.getCurrentPosition();
            double leftTarget = j - distance;
            double rightTarget = i - distance;



            while (opModeIsActive() && i > rightTarget && j > leftTarget) {
                if(i> rightTarget){
                    robot.right.setPower(-(maxRightSpeed * speed));
                    i = robot.right.getCurrentPosition();

                }
                if(j>leftTarget) {
                    robot.left.setPower(-(maxLeftSpeed * speed));
                    j = robot.left.getCurrentPosition();

                }


            }

            hardBrake();

        }
    }

    /**
     * Turns the robot left based on the given parameters speed and seconds.
     * @param speed
     * @param distance
     */
    public void turnLeft(double speed, double distance){
        double maxLeftSpeed = 0.7;
        double maxRightSpeed = 1;

        // robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {
            double i = robot.right.getCurrentPosition();
            double j = robot.left.getCurrentPosition();
            double leftTarget = j + distance;
            double rightTarget = i - distance;


            while (opModeIsActive() && i > rightTarget && j < leftTarget) {
                if(i > rightTarget){
                    robot.right.setPower(-(maxRightSpeed * speed));
                    i = robot.right.getCurrentPosition();

                }
                if(j < leftTarget) {
                    robot.left.setPower((maxLeftSpeed * speed));
                    j = robot.left.getCurrentPosition();

                }


            }

            robot.left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.left.setPower(-1);
            robot.right.setPower(1);
            sleep(100);
            robot.left.setPower(0);
            robot.right.setPower(0);        }
    }

    /**
     * Turns the robot right based on the given parameters speed and seconds.
     * @param speed
     * @param distance
     */
    public void turnRight(double speed, double distance){
        double maxLeftSpeed = 0.7;
        double maxRightSpeed = 1;

        // robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {
            double i = robot.right.getCurrentPosition();
            double j = robot.left.getCurrentPosition();
            double leftTarget = j - distance;
            double rightTarget = i + distance;


            while (opModeIsActive() && i < rightTarget && j > leftTarget) {
                if(i < rightTarget){
                    robot.right.setPower((maxRightSpeed * speed));
                    i = robot.right.getCurrentPosition();

                }
                if(j>leftTarget) {
                    robot.left.setPower(-(maxLeftSpeed * speed));
                    j = robot.left.getCurrentPosition();

                }


            }

            robot.left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.left.setPower(1);
            robot.right.setPower(-1);
            sleep(100);
            robot.left.setPower(0);
            robot.right.setPower(0);        }
    }

    public  void hardBrake() {
        robot.left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.left.setPower(1);
        robot.right.setPower(1);
        sleep(200);
        robot.left.setPower(0);
        robot.right.setPower(0);
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
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
