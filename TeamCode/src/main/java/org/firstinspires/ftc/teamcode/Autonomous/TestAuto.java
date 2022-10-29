package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.DashConstants.Dash_Vision.currentDuckPos;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
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

        Trajectory traj1 = robot.drivetrain.trajectoryBuilder(new Pose2d())
                .forward(28)
                .build();

        Trajectory traj2 = robot.drivetrain.trajectoryBuilder(traj1.end())
                .strafeRight(35)
                .build();
        Trajectory traj3 = robot.drivetrain.trajectoryBuilder(new Pose2d())
                .back(28)
                .build();

        Trajectory traj4 = robot.drivetrain.trajectoryBuilder(traj1.end())
                .strafeLeft(35)
                .build();


        multTelemetry.addLine("Waiting for start");
        multTelemetry.addData("Duck Pos", currentDuckPos);
        multTelemetry.update();




        waitForStart();

        if (opModeIsActive()) {

            robot.drivetrain.followTrajectory(traj1);
            robot.drivetrain.followTrajectory(traj2);
            robot.drivetrain.followTrajectory(traj3);
            robot.drivetrain.followTrajectory(traj4);

        }
    }
}

