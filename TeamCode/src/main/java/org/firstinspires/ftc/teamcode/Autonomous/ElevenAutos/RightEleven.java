package org.firstinspires.ftc.teamcode.Autonomous.ElevenAutos;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@Disabled
@Autonomous(name="Right Eleven", group="Autonomous Linear Opmode")
public class RightEleven extends LinearOpMode {
    Robot robot;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public SignalPipeline.SignalSide signalSide;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
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



    public void initialize() {
        setOpMode(this);
        robot = new Robot(true);
        Side.setBlue();
        robot.scorer.autoStart();
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

        Pose2d startPos = new Pose2d(-40,60,Math.toRadians(90));
        robot.drivetrain.setPoseEstimate(startPos);



        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()
        Trajectory setUp = robot.drivetrain.trajectoryBuilder(startPos)
                .strafeRight(17)
                .build();

        Trajectory setUp2 = robot.drivetrain.trajectoryBuilder(setUp.end())
                .back(38)
                .build();

        Trajectory park1 = robot.drivetrain.trajectoryBuilder(setUp2.end())
                .strafeRight(34)
                .build();

        Trajectory park3 = robot.drivetrain.trajectoryBuilder(setUp2.end())
                .strafeLeft(34)
                .build();

        Trajectory rotate1 = robot.drivetrain.trajectoryBuilder(park1.end())
                .splineTo(new Vector2d(robot.drivetrain.getPoseEstimate().getX(),robot.drivetrain.getPoseEstimate().getY()),90)
                .build();

        Trajectory rotate2 = robot.drivetrain.trajectoryBuilder(setUp2.end())
                .splineTo(new Vector2d(robot.drivetrain.getPoseEstimate().getX(),robot.drivetrain.getPoseEstimate().getY()),90)
                .build();

        Trajectory rotate3 = robot.drivetrain.trajectoryBuilder(park3.end())
                .splineTo(new Vector2d(robot.drivetrain.getPoseEstimate().getX(),robot.drivetrain.getPoseEstimate().getY()),90)
                .build();


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

//                telemetry.update();
            }

            sleep(20);
        }

        if (opModeIsActive()) {

            multTelemetry.addData("signal side", signalSide);
            multTelemetry.update();


            if (signalSide == ONE){
                robot.drivetrain.followTrajectory(setUp);
                robot.drivetrain.followTrajectory(setUp2);
                robot.drivetrain.followTrajectory(park1);
                robot.drivetrain.updatePoseEstimate();
                robot.drivetrain.followTrajectory(rotate1);
            }else if (signalSide == TWO){
                robot.drivetrain.followTrajectory(setUp);
                robot.drivetrain.followTrajectory(setUp2);
                robot.drivetrain.updatePoseEstimate();
                robot.drivetrain.followTrajectory(rotate2);
            }else if (signalSide == THREE){
                robot.drivetrain.followTrajectory(setUp);
                robot.drivetrain.followTrajectory(setUp2);
                robot.drivetrain.followTrajectory(park3);
                robot.drivetrain.updatePoseEstimate();
                robot.drivetrain.followTrajectory(rotate3);
            }



            multTelemetry.addData("signal side", signalSide);
            multTelemetry.addData("ending auto", "ok");
            multTelemetry.update();

        }
    }
}
