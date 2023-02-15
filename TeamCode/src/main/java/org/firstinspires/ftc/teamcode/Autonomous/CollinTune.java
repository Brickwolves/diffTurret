package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.CollinTune.MoveTune.x;
import static org.firstinspires.ftc.teamcode.Autonomous.CollinTune.MoveTune.y;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffset;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.proportionalWeight;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.ONE;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.THREE;
import static org.firstinspires.ftc.teamcode.Vision.SignalPipeline.SignalSide.TWO;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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


@Autonomous(name="CollinTuning", group="Autonomous Linear Opmode")
public class CollinTune extends LinearOpMode {

    public static class MoveTune{
        public static double x;
        public static double y;
    }

    public FtcDashboard dashboard = FtcDashboard.getInstance();

    Robot robot;
    private PID rotationPID;
    private CollinMovement movement;
    private ElapsedTime runtime = new ElapsedTime();

    public void initialize() {
        setOpMode(this);
        robot = new Robot(true);
        robot.scorer.autoStart();
        movement = new CollinMovement(robot);
        rotationPID = new PID(proportionalWeight, integralWeight, derivativeWeight);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();

        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()
        Pose2d startPos = new Pose2d(30,60,Math.toRadians(90));
        robot.drivetrain.setPoseEstimate(startPos);

        while (opModeIsActive()) {

            robot.drivetrain.update();
            robot.drivetrain.getPoseEstimate();

            robot.drivetrain.setDrivePower(
                    movement.tuneDrive(x - robot.drivetrain.getPoseEstimate().getX()),
                    movement.tuneStrafe(y - robot.drivetrain.getPoseEstimate().getY()),
                    rotationPID.update(robot.gyro.getAngle() - 0, true), 1);

            multTelemetry.addData("x", robot.drivetrain.getPoseEstimate().getX());
            multTelemetry.addData("y", robot.drivetrain.getPoseEstimate().getY());
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .setFill("black").fillCircle(robot.drivetrain.getPoseEstimate().getX(), robot.drivetrain.getPoseEstimate().getY(), 1);
            dashboard.sendTelemetryPacket(packet);
            multTelemetry.update();

        }
    }
}
