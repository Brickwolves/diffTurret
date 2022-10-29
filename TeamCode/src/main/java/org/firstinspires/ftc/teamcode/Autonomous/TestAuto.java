package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Vision.currentDuckPos;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.speed1;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.speed2;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.speed3;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.speed4;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.speed5;
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
import org.firstinspires.ftc.teamcode.Odometry.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Utilities.Loggers.Side;


@Autonomous(name="Test Auto", group="Autonomous Linear Opmode")
public class TestAuto extends LinearOpMode {
    Robot robot;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

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
        Trajectory traj1 = robot.drivetrain.trajectoryBuilder(new Pose2d(-36,70,Math.toRadians(270)))
                .splineTo(new Vector2d(-36, 15), Math.toRadians(270),
                        regulateSpeed1(speed1),
                        regulateSpeed2())
                .splineTo(new Vector2d(-50, 15), Math.toRadians(180),
                        regulateSpeed1(speed2),
                        regulateSpeed2())
                .splineTo(new Vector2d(-15, 15), Math.toRadians(90),
                        regulateSpeed1(speed4),
                        regulateSpeed2())
                .splineTo(new Vector2d(-10, 25), Math.toRadians(0),
                        regulateSpeed1(speed5),
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

