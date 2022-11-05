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
public class NovAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
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

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aadilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    tagToTelemetry(tagOfInterest);
                }
                else
                {

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nwe HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {

                if(tagOfInterest == null)
                {
                }
                else
                {
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
            driveBack(0.8,32);
            sleep(100);
            turnRight(.5, 40);
            sleep(1000);
            moveArm(0.8, 324, 310);
            driveStraight(.9, 15);
            sleep(1000);
            robot.servo.setPosition(0);
            sleep(1000);
            driveBack(0.5, 21);
            tagToTelemetry(tagOfInterest);
            if (tagOfInterest.id == MIDDLE){

            }
            if (tagOfInterest.id == LEFT){

            }
            if (tagOfInterest.id == RIGHT){

            }

//            sleep(1000);
//            driveBack(.5, 21);
//            turnLeft(.5, 30);
        }

    }

    public void driveStraight(double speed, double seconds){
        double maxLeftSpeed = 1;
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
        double maxLeftSpeed = 1;
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
        double maxLeftSpeed = 1;
        double maxRightSpeed = 0.9;
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
        double maxLeftSpeed = 1;
        double maxRightSpeed = 0.9;
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
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
