package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreBack;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bStartAuto;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.ONE;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.THREE;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.TWO;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.V4BUpdater;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.SignalPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(name="CURSED AUTO (left)", group="Autonomous Linear Opmode")
public class CursedAutoRight extends LinearOpMode {
    Robot robot;
    V4BUpdater specialV4B;
    OpenCvCamera camera;
    public Trajectory midPreloadRight1;
    public Trajectory midPreloadRight2;
    public Trajectory lowCycleRight1;
    public Trajectory lowCycleRight2;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public Pose2d startRight;
    public Pose2d cycleRightStart;

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
        specialV4B = new V4BUpdater(robot.scorer);

        Side.setBlue();
        robot.scorer.autoStart();

        //Trajectories for preload
        startRight = new Pose2d(-40,62,Math.toRadians(90));

        midPreloadRight1 = robot.drivetrain.trajectoryBuilder(startRight)
                .lineToConstantHeading(new Vector2d(-35,45))
                .build();

        midPreloadRight2 = robot.drivetrain.trajectoryBuilder(midPreloadRight1.end())
                .lineToConstantHeading(new Vector2d(-25,26))
                .build();

        lowCycleRight1 = robot.drivetrain.trajectoryBuilder(midPreloadRight2.end())
                .lineToConstantHeading(new Vector2d(-47,16))
                .build();

        //Trajectories for Cycle
        cycleRightStart = new Pose2d((-47,62,Math.toRadians(150));

        lowCycleRight2  = robot.drivetrain.trajectoryBuilder(cycleRightStart)
                .lineToConstantHeading(new Vector2d(-60,16));
                .build();

    }

    public void preloadMidRight(){
        robot.drivetrain.followTrajectory(midPreloadRight1);
        robot.scorer.grabber(grabberDown);
        robot.drivetrain.followTrajectory(midPreloadRight2);
        robot.scorer.autoMid();
        specialV4B.setTarget(v4bScoreBack);
        robot.drivetrain.turnTo(Math.toRadians(45));
        robot.scorer.sleep(2);
        robot.scorer.autoDeposit();
        specialV4B.setTarget(v4bDown);
        robot.drivetrain.followTrajectory(lowCycleRight1);
        robot.drivetrain.turnTo(Math.toRadians(150));
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





        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()
        Pose2d startPos = new Pose2d(30,60,Math.toRadians(90));
        robot.drivetrain.setPoseEstimate(startPos);


        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()
//        Trajectory preloadSetup1 = robot.drivetrain.trajectoryBuilder(startPos)
//                .lineToConstantHeading(new Vector2d(35,45))
//                .build();
//
//        Trajectory preloadSetup2 = robot.drivetrain.trajectoryBuilder(preloadSetup1.end())
//                .lineToConstantHeading(new Vector2d(26,27))
//                .build();
//        Trajectory cycleSetup1 = robot.drivetrain.trajectoryBuilder(preloadSetup2.end())
//                .lineToConstantHeading(new Vector2d(50,15))
//                .build();
//        Trajectory cycleSetup2 = robot.drivetrain.trajectoryBuilder(cycleSetup1.end())
//                .lineToConstantHeading(new Vector2d(58,15))
//                .build();
//        Trajectory cycleSetup3 = robot.drivetrain.trajectoryBuilder(cycleSetup2.end())
//                .lineToConstantHeading(new Vector2d(50,23))
//                .build();
//        Trajectory cycleSetup4 = robot.drivetrain.trajectoryBuilder(cycleSetup3.end())
//                .lineToConstantHeading(new Vector2d(58,15))
//                .build();





        while(opModeInInit()){
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

        if (opModeIsActive()) {
            specialV4B.setTarget(v4bStartAuto);
            specialV4B.start();
            preloadMidRight();
            robot.cycleLowRight(5);
            if (signalSide == ONE) {
                robot.drivetrain.followTrajectory(robot.park1Left);
            }else if(signalSide == TWO){
                robot.drivetrain.followTrajectory(robot.park2Left);
            }else if (signalSide == THREE){
                robot.drivetrain.followTrajectory(robot.lowCycleLeft4);
            }


            multTelemetry.addData("signal side", signalSide);
            multTelemetry.addData("ending auto", "ok");
            multTelemetry.update();

            specialV4B.exit();

        }
    }
}