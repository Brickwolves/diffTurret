package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive.regulateSpeed1;
import static org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive.regulateSpeed2;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.SignalPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name="Left Auto Wait", group="Autonomous Linear Opmode")
public class LeftAutoWait extends LinearOpMode {
    Robot robot;
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
        Side.setRed();

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {


        initialize();

        robot = new Robot();

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

        robot.drivetrain.setPoseEstimate(new Pose2d(-40,-70,Math.toRadians(90)));



        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()
        Trajectory traj1 = robot.drivetrain.trajectoryBuilder(new Pose2d(-40,-70,Math.toRadians(90)))
                .splineTo(new Vector2d(-36, -50),Math.toRadians(90),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(-20, -50),Math.toRadians(90),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .build();

        Trajectory traj2 = robot.drivetrain.trajectoryBuilder(new Pose2d(-40,-70,Math.toRadians(90)))
                .splineTo(new Vector2d(-36, -50),Math.toRadians(90),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .build();

        Trajectory traj3 = robot.drivetrain.trajectoryBuilder(new Pose2d(-40,-70,Math.toRadians(90)))
                .splineTo(new Vector2d(-36, -50),Math.toRadians(90),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(-60, -50),Math.toRadians(90),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .build();

        while(opModeInInit()){
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

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
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                    }
                }

                telemetry.update();
            }

            sleep(20);
        }


        if (opModeIsActive()) {


            multTelemetry.addData("signal side", signalSide);
            multTelemetry.update();

            if (signalSide == ONE){
                robot.drivetrain.followTrajectory(traj1);
            }
            if (signalSide == TWO){
                robot.drivetrain.followTrajectory(traj2);
            }
            if (signalSide == THREE){
                robot.drivetrain.followTrajectory(traj3);
            }

            multTelemetry.addData("signal side", signalSide);
            multTelemetry.addData("ending auto", "ok");
            multTelemetry.update();


        }
    }
}