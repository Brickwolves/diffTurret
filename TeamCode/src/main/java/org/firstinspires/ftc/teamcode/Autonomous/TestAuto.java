package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive.regulateSpeed1;
import static org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive.regulateSpeed2;
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

import org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Vision.Camera;
import org.firstinspires.ftc.teamcode.Odometry.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Utilities.Loggers.Side;
import org.firstinspires.ftc.teamcode.VisionPipelines.SignalPipeline;


@Autonomous(name="Test Auto", group="Autonomous Linear Opmode")
public class TestAuto extends LinearOpMode {
    Robot robot;
    Camera camera;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public SignalPipeline.SignalSide signalSide;


    public void initialize() {
        setOpMode(this);
        Side.setBlue();

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {


        initialize();

        robot = new Robot();


        robot.drivetrain.setPoseEstimate(new Pose2d(-36,66,Math.toRadians(270)));


        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()
        Trajectory traj1 = robot.drivetrain.trajectoryBuilder(new Pose2d(-30,70,Math.toRadians(270)))
                .lineTo(new Vector2d(-38, 65),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(-36, 20), Math.toRadians(270),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(-50, 20), Math.toRadians(180),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(-15, 20), Math.toRadians(90),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(-10, 30), Math.toRadians(0),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .build();
//
//        Trajectory traj2 = robot.drivetrain.trajectoryBuilder(new Pose2d(robot.drivetrain.getPoseEstimate().getX(),robot.drivetrain.getPoseEstimate().getY(),robot.drivetrain.getExternalHeading()))
//                .strafeRight(20)
//                .build();
//        Trajectory traj3 = robot.drivetrain.trajectoryBuilder(new Pose2d())
//                .back(28)
//                .build();
//
//        Trajectory traj4 = robot.drivetrain.trajectoryBuilder(traj1.end())
//                .strafeLeft(35)
//                .build();






        waitForStart();

        if (opModeIsActive()) {

            robot.drivetrain.followTrajectory(traj1);
           // robot.drivetrain.followTrajectory(traj2);
//            robot.drivetrain.followTrajectory(traj3);
//            robot.drivetrain.followTrajectory(traj4);

        }
    }
}

