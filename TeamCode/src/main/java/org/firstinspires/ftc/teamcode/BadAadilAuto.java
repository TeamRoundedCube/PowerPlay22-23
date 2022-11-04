package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BadAadilAuto extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aadilTagDetectionPipeline;
    AadilHardwareFullBot robot = new AadilHardwareFullBot();

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
    public void runOpMode() {
     /*   int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aadilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aadilTagDetectionPipeline);
        robot.init(hardwareMap);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        if (!opModeIsActive()) {
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
                    telemetry.update();
                }
                else
                {

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                        telemetry.update();
                    }
                    else
                    {
                        telemetry.addLine("\nwe HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                        telemetry.update();
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
                    telemetry.update();
                }

            }

            telemetry.update();
            sleep(20);
        }

      */
        waitForStart();

        while (opModeIsActive()){

            //Detect cone
            driveBack(1, 15);
            sleep(1000);
            //Go straight
            driveBack(1, 15);
            //Turn and drop

            // if left {

            //}
            //else if(right) {

            //}
            // else {

            //}

         /*   ArrayList<AprilTagDetection> currentDetections = aadilTagDetectionPipeline.getLatestDetections();
            //location 2
            driveBack(1, 15);
            for(AprilTagDetection tag : currentDetections) {

                if(tag.id == LEFT){
                    turnLeft(.25, 1.2);
                    driveStraight(1,15);
                }
                else if (tag.id == RIGHT){
                    turnLeft(.25, 1.2);
                    driveBack(1,15);
                }
                else{
                    robot.right.setPower(0);
                    robot.left.setPower(0);

                }
            }
            robot.right.setPower(0);
            robot.left.setPower(0);*/
        }
    }

// Functions

    public void driveStraight(double speed, double seconds){
        robot.left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds * 1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds) {

                robot.right.setPower(speed);
                robot.left.setPower(speed);

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
        robot.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds * 1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds) {

                robot.right.setPower(-(0.7*speed));
                robot.left.setPower(-speed);

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
        robot.left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds * 1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds) {

                robot.right.setPower(-speed);
                robot.left.setPower(speed);

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
        robot.left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {

            runtime.reset();
            double milliseconds = seconds * 1000;
            double i = runtime.milliseconds();
            while (opModeIsActive() && i < milliseconds) {

                robot.right.setPower(speed);
                robot.left.setPower(-speed);

                i++;
            }

            robot.left.setPower(0);
            robot.right.setPower(0);

        }
    }
//    void tagToTelemetry(AprilTagDetection detection) {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//    }
}
