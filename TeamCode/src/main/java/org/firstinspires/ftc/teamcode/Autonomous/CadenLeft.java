package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Hardware.Scoring.idfk;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.imuOffset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffsetAuto;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.v4bOffset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.v4bOffsetAuto;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.proportionalWeight;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.ONE;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.THREE;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.TWO;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Control.Hardware.V6Hardware;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.SignalPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name="Left 1+2", group="Autonomous Linear Opmode")
public class CadenLeft extends LinearOpMode {
    Robot robot;

    public PID pid;

    public ElapsedTime timeout = new ElapsedTime();


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

    Movement drive;

    double[][] preloadFromStart = {{0, 0}, {18, -26}, {18, -64}, {0, -84}};
    double[][] wallSetUpFromPreload = {{0, -84}, {20, -102}};
    double[][] wallFromWallSetUp = {{20, -102}, {20, -133}, {63, -133}};
    double[][] stackFromWall = {{63, -133},{93,-133}};
    double[][] cycleFromStack = {{93,-133}, {9,-133},{13, -118}};
    double[][] park2FromCycle = {{13, -118}, {21, -136}};
    double[][] park1FromCycle = {{21,-136}, {73, -136}};
    double[][] park3FromCycle = {{21,-136}, {-43, -136}};
    double[][] wallFromCycle = {{13, -118}, {63, -133}};


    Movement.Function[] preloadScore, goToWall, cycle, parkSignal1, parkSignal2, parkSignal3, goToWallCycle, approachStack, goToWallSetUp;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public SignalPipeline.SignalSide signalSide;



    public void initialize() {
        setOpMode(this);
        robot = new Robot(true);

        Side.setBlue();
        robot.scorer.autoStart();

        V6Hardware hardware = new V6Hardware(hardwareMap);
        drive = new Movement(0,0,hardware);

        pid = new PID(proportionalWeight, integralWeight, derivativeWeight);


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

        buildPaths();

        while(opModeInInit()){
            slidesOffset = 0;
            v4bOffset = 0;
            imuOffset = 0;
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
            robot.scorer.autoStart();
        }

    }

    public void midCycleAutos() {
        //SET UP TO SCORE PRELOAD
        double heading = 0;
        while(opModeIsActive() && drive.followPath(preloadScore, 0.7, heading, .65,.005, 2, .35, true, true)){
            if(drive.t > 0.5){
                heading = .69;
                robot.scorer.braceOut();
                robot.scorer.autoMid();
            }
        }
        robot.scorer.sleep(0.6,drive,0,-84,.69);
        //SCORE PRELOAD ON MID
        robot.scorer.autoDeposit(drive,0,-84,.69);
        int stackHeight = 5;
        while (stackHeight > 1) {
            if (stackHeight == 5) {
                //MOVE TO WALL FROM SCORE PRELOAD - FIRST CYCLE
                while(opModeIsActive() && drive.followPath(goToWallSetUp, 0.65, 1.57, .85,.005, 2000, .35, false, false)) {
                }
                while(opModeIsActive() && drive.followPath(goToWall, 0.6, 1.57, .85,.005, 2000, .35, false, false)){
                    if(drive.t > 0.5){
                        robot.scorer.stackPickup(stackHeight);
                        robot.scorer.open(false);
                    }
                }

            } else {
                //MOVE TO WALL FROM CYCLE - LATER CYCLES
                while(opModeIsActive() && drive.followPath(goToWallCycle, 0.6, 1.57, .85,.005, 2000, .35, false, false)){
                    if(drive.t > 0.4) {
                        robot.scorer.stackPickup(stackHeight);
                        robot.scorer.open(false);
                    }
                }
            }
            //TODO DELETE
//            robot.scorer.sleep(1,drive,63,-133,1.57);
            //DRIVE TO WALL WHILE WAITING FOR BREAK BEAMS
            double power = .35;
            while(opModeIsActive() && drive.followPath(approachStack, power, 1.57, 1,.005, 5000, .25, true, false)){
                if(drive.t > .2){
                    power = .3;
                }
                robot.scorer.slidesHold();
                robot.scorer.v4bHold();
                if(robot.scorer.beamBroken()){
                    drive.stopDrive();
                    break;
                }
            }
            //STACK ESCAPE HEIGHT
            robot.scorer.close();
            robot.scorer.sleep(0.8,drive.odo);
            robot.scorer.stackEscape(stackHeight);
            robot.scorer.sleep(0.5,drive.odo);
            //GO TO MID AND SCORE
            double cycleHeading =1.57;
            while(opModeIsActive() && drive.followPath(cycle, 0.5, cycleHeading, .75,.005, 2, .35, true, false)){
                if(drive.t>0.7){
                    cycleHeading = 2.19;
                    robot.scorer.autoMid();
                }
            }
            robot.scorer.autoMid();
            robot.scorer.sleep(0.8,drive,13,-118,2.19);
            robot.scorer.autoDeposit(drive,13,-118,2.19);
            //TODO DELETE
            robot.scorer.sleep(1,drive,13,-118,2.19);
            stackHeight--;
        }
    }


    public void setUpPark(){
    }



    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        if (opModeIsActive()) {
            midCycleAutos();
//            if (signalSide == ONE) {
//                robot.drivetrain.followTrajectorySequence(moveToWallThird);
//                robot.drivetrain.turn(Math.toRadians(90));
//            }else if(signalSide == TWO){
//                robot.drivetrain.followTrajectorySequence(park2);
//                robot.drivetrain.turn(Math.toRadians(135));
//            }else if (signalSide == THREE){
//                robot.drivetrain.followTrajectorySequence(park2);
//                robot.drivetrain.followTrajectorySequence(park3);
//                robot.drivetrain.turn(Math.toRadians(135));
//            }


            multTelemetry.addData("signal side", signalSide);
            multTelemetry.addData("ending auto", "ok");
            multTelemetry.addData("idfk",idfk);
            multTelemetry.update();

            slidesOffsetAuto = robot.scorer.getHeight();
            v4bOffsetAuto = robot.scorer.v4b.getCurrentPosition();
            imuOffset = robot.gyro.getAngle();

        }
        multTelemetry.addData("idfk",idfk);
        multTelemetry.update();
    }

    private void buildPaths(){
        preloadScore = drive.getBezierCurve(preloadFromStart);
        goToWall = drive.getBezierCurve(wallFromWallSetUp);
        goToWallSetUp = drive.getLine(wallSetUpFromPreload);
        cycle = drive.getLine(cycleFromStack);
        parkSignal2 = drive.getLine(park2FromCycle);
        parkSignal1 = drive.getLine(park1FromCycle);
        parkSignal3 = drive.getLine(park3FromCycle);
        goToWallCycle = drive.getLine(wallFromCycle);
        approachStack = drive.getLine(stackFromWall);
    }
}