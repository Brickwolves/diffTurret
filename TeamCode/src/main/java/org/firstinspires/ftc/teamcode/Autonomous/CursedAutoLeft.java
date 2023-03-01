package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bStartAuto;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.v4bOffset;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.proportionalWeight;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.ONE;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.STOOPID;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.THREE;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.TWO;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.SignalPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name="CURSED AUTO (left)", group="Autonomous Linear Opmode")
public class CursedAutoLeft extends LinearOpMode {
    Robot robot;
    public Trajectory midPreloadLeft1;
    public Trajectory midPreloadLeft2;
    public TrajectorySequence moveToWallFive;
    public TrajectorySequence moveToWallNotFive;
    public TrajectorySequence driveToPreloadPole;

    public TrajectorySequence lowCycleLeft2;
    public Trajectory lowCycleLeft3;
    public Trajectory lowCycleLeft4;

    public Pose2d startLeft;
    public Pose2d postRotateLeft;

    public TrajectorySequence park3;
    public TrajectorySequence park2;

    public PID pid;


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public SignalPipeline.SignalSide signalSide;



    public void initialize() {
        setOpMode(this);
        robot = new Robot(true);



        Side.setBlue();
        robot.scorer.autoStart();

        pid =new PID(proportionalWeight,integralWeight,derivativeWeight);

        startLeft = new Pose2d(30,60,Math.toRadians(90));

        postRotateLeft = new Pose2d(37,47,45);

        robot.drivetrain.setPoseEstimate(startLeft);

        driveToPreloadPole = robot.drivetrain.trajectorySequenceBuilder(startLeft)
                .lineToLinearHeading(new Pose2d(37,37,45))
//                .turn(Math.toRadians(-45))
                .addTemporalMarker(() -> robot.scorer.autoMid())
                .lineToConstantHeading(new Vector2d(31,22))
                .build();
        moveToWallFive = robot.drivetrain.trajectorySequenceBuilder(driveToPreloadPole.end())
                .addTemporalMarker(() -> robot.scorer.braceIn())
                .lineToLinearHeading(new Pose2d(23,22,0))
                .lineToConstantHeading(new Vector2d(23,-5))
                .addTemporalMarker(() -> robot.scorer.braceOut())
                .lineToConstantHeading(new Vector2d(40,-5))
                .build();

        waitForStart();

    }

    public void midCycleAutos(){
        /*
        robot.drivetrain.followTrajectory(midPreloadLeft1);
        robot.drivetrain.turnTo(Math.toRadians(45));
        robot.scorer.grabber(grabberDown);
        robot.drivetrain.followTrajectory(midPreloadLeft2);
sy
         */
        robot.scorer.braceOut();

        robot.scorer.v4b(v4bStartAuto);
        robot.drivetrain.update();
        robot.drivetrain.followTrajectorySequenceAsync(driveToPreloadPole);
        while(robot.drivetrain.isBusy() && opModeIsActive()){
            robot.drivetrain.update();
            robot.scorer.v4bHold();
        }
        robot.scorer.sleep(0.2);
        robot.scorer.autoDeposit();
        int stackheight = 5;
        while(stackheight > 3) {
            robot.scorer.grabber(grabberDown);
            if(stackheight == 5) {
                robot.drivetrain.followTrajectorySequenceAsync(moveToWallFive);
//                robot.drivetrain.turnTo(0);

                multTelemetry.addData("angle", robot.drivetrain.getPoseEstimate().getHeading());
                multTelemetry.update();
            }else{
                moveToWallNotFive = robot.drivetrain.trajectorySequenceBuilder(lowCycleLeft2.end())
                        .lineToConstantHeading(new Vector2d(35,0))
                        //This line and the two lines after it are alternatives, don't run both
                        .turn(Math.toRadians(45))
                        .lineToConstantHeading(new Vector2d(35,-3))
                        .build();
                robot.drivetrain.followTrajectorySequenceAsync(moveToWallNotFive);
            }
            robot.scorer.open(false);
            while (robot.drivetrain.isBusy() && opModeIsActive()) {
                robot.drivetrain.update();
                robot.scorer.v4bHold();
            }
            robot.scorer.stackPickup(stackheight);
            robot.scorer.open(false);
            robot.scorer.sleep(0.5);
            robot.drivetrain.setDrivePower(1, 0, 0, 0.4);
            while (!robot.scorer.beamBroken()) {
                robot.scorer.v4bHold();
                robot.drivetrain.update();
                robot.scorer.updateBeam();
            }
            robot.scorer.sleep(0.1);
            robot.drivetrain.setDrivePower(0, 0, 0, 0);
            robot.scorer.close();
            robot.scorer.sleep(0.3);
            robot.scorer.stackEscape(stackheight);
            stackheight = stackheight - 1;
            robot.drivetrain.update();
            lowCycleLeft2 = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(25,0))
                    .turn(Math.toRadians(-45))
                    .addTemporalMarker(() -> robot.scorer.autoMid())
                    .lineToConstantHeading(new Vector2d(7,5))
                    .build();
            robot.drivetrain.followTrajectorySequenceAsync(lowCycleLeft2);
            while (robot.drivetrain.isBusy() && opModeIsActive()) {
                robot.drivetrain.update();
                robot.scorer.v4bHold();
            }
            robot.scorer.autoDeposit();
        }
        park2 = robot.drivetrain.trajectorySequenceBuilder(lowCycleLeft2.end())
                .lineToConstantHeading(new Vector2d(15,0))
                .build();
        park3 = robot.drivetrain.trajectorySequenceBuilder(park2.end())
                .lineToConstantHeading(new Vector2d(0,0))
                .build();

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {


        initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        robot.drivetrain.setPoseEstimate(startLeft);

        while(opModeInInit()){
            v4bOffset = 0;
            robot.scorer.v4bNeutral();
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if(detections != null)
            {
                //telemetry.addData("FPS", camera.getFps());
                //telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                //telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        if(detection.id == 4){
                            signalSide = TWO;
                        }
                        if(detection.id == 3){
                            signalSide = THREE;
                        }
                        if(detection.id == 18){
                            signalSide = ONE;
                        }

                        multTelemetry.addData("Signal", signalSide);
                        multTelemetry.update();
//                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                    }
                }

                //telemetry.update();
            }


            sleep(20);
        }

        robot.scorer.autoStart();




        if (opModeIsActive()) {
            midCycleAutos();


            if (signalSide == ONE) {
                robot.drivetrain.followTrajectorySequence(moveToWallNotFive);
                robot.drivetrain.turn(Math.toRadians(90));
            }else if(signalSide == TWO){
                robot.drivetrain.followTrajectorySequence(park2);
                robot.drivetrain.turn(Math.toRadians(135));
            }else if (signalSide == THREE){
                robot.drivetrain.followTrajectorySequence(park2);
                robot.drivetrain.followTrajectorySequence(park3);
                robot.drivetrain.turn(Math.toRadians(135));
            }


            multTelemetry.addData("signal side", signalSide);
            multTelemetry.addData("ending auto", "ok");
            multTelemetry.update();

            slidesOffset = robot.scorer.getHeight();
            v4bOffset = robot.scorer.v4b.getCurrentPosition();

        }
    }
}