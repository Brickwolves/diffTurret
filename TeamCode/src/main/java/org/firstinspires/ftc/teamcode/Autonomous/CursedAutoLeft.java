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

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.SignalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="CURSED AUTO (left)", group="Autonomous Linear Opmode")
public class CursedAutoLeft extends LinearOpMode {
    Robot robot;
    OpenCvCamera camera;
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

    public Trajectory park1Left;
    public Trajectory park2Left;

    public PID pid;



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
                .lineToConstantHeading(new Vector2d(37,40))
                .turn(Math.toRadians(-45))
                .addTemporalMarker(() -> robot.scorer.autoMid())
                .lineToConstantHeading(new Vector2d(29,16))
                .build();
        moveToWallFive = robot.drivetrain.trajectorySequenceBuilder(driveToPreloadPole.end())
                .lineToConstantHeading(new Vector2d(32,22))
                .lineToConstantHeading(new Vector2d(35,0))
                .turn(Math.toRadians(315))
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
        robot.scorer.sleep(1);
        robot.scorer.autoDeposit();
        int stackheight = 5;
        while(stackheight > 0) {
            robot.scorer.grabber(grabberDown);
            if(stackheight == 5) {
                robot.drivetrain.followTrajectorySequenceAsync(moveToWallFive);
            }else{
                moveToWallNotFive = robot.drivetrain.trajectorySequenceBuilder(lowCycleLeft2.end())
                        .lineToConstantHeading(new Vector2d(25,13))
                        .turn(Math.toRadians(270))
                        .lineToConstantHeading(new Vector2d(35,10))
                        .build();
                robot.drivetrain.followTrajectorySequenceAsync(moveToWallNotFive);
            }
            robot.scorer.open(false);
            while (robot.drivetrain.isBusy() && opModeIsActive()) {
                robot.drivetrain.update();
                robot.scorer.v4bHold();
            }
            robot.scorer.stackPickup(stackheight - 3);
            robot.scorer.open(false);
            robot.drivetrain.setDrivePower(1, 0, 0, 0.2);
//            while (!robot.scorer.beamBroken()) {
//                robot.scorer.v4bHold();
//                robot.drivetrain.update();
//                robot.scorer.updateBeam();
//                multTelemetry.addData("breakbeams", robot.scorer.beamBroken());
//                multTelemetry.update();
//            }
//            multTelemetry.addData("breakbeams", robot.scorer.beamBroken());
//            multTelemetry.update();
            robot.drivetrain.setDrivePower(0, 0, 0, 0);
            robot.scorer.close();
            robot.scorer.stackEscape(stackheight);
            stackheight = stackheight - 1;
            robot.drivetrain.update();
            lowCycleLeft2 = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(26,9))
                    .turn(Math.toRadians(360 - 225))//THIS IS WACKY
                    .build();
            robot.drivetrain.followTrajectorySequenceAsync(lowCycleLeft2);
            while (robot.drivetrain.isBusy() && opModeIsActive()) {
                robot.drivetrain.update();
                robot.scorer.v4bHold();
            }
            robot.scorer.autoMid();
            robot.scorer.sleep(1);
            robot.scorer.autoDeposit();
        }





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

        /*
        while(opModeInInit()){
            v4bOffset = 0;
            robot.scorer.v4bNeutral();
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if(detections != null)
            {
//                telemetry.addData("FPS", camera.getFps());
//                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

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

                telemetry.update();
            }


            sleep(20);
            robot.scorer.autoStart();
        }

         */

        if (opModeIsActive()) {
            multTelemetry.addData("breakbeams", robot.scorer.beamBroken());
            midCycleAutos();

//            robot.cycleLowLeft(5);
//            if (signalSide == ONE) {
//                robot.drivetrain.followTrajectory(robot.park1Left);
//            }else if(signalSide == TWO){
//                robot.drivetrain.followTrajectory(robot.park2Left);
//            }else if (signalSide == THREE){
//                robot.drivetrain.followTrajectory(robot.lowCycleLeft4);
//            }


            multTelemetry.addData("signal side", signalSide);
            multTelemetry.addData("ending auto", "ok");
            multTelemetry.update();

            slidesOffset = robot.scorer.getHeight();
            v4bOffset = robot.scorer.v4b.getCurrentPosition();

        }
    }
}