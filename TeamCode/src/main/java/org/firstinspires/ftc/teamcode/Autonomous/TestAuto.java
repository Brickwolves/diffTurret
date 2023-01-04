package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive.regulateSpeed1;
import static org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive.regulateSpeed2;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera;
import org.firstinspires.ftc.teamcode.Utilities.Loggers.Side;
import org.firstinspires.ftc.teamcode.Vision.SignalPipeline;


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


        robot.drivetrain.setPoseEstimate(new Pose2d(-60,60,Math.toRadians(270)));


        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()
        Trajectory traj1 = robot.drivetrain.trajectoryBuilder(new Pose2d(-60,60,Math.toRadians(270)))
                .splineTo(new Vector2d(-60, -45),Math.toRadians(270),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(0, -45),Math.toRadians(270),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(45,-60),Math.toRadians(270),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(60, 0),Math.toRadians(270),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(60,45),Math.toRadians(270),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(-0, 60
                        ),Math.toRadians(270),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .splineTo(new Vector2d(-45,60),Math.toRadians(270),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .build();






        waitForStart();

        if (opModeIsActive()) {

            robot.drivetrain.followTrajectory(traj1);


        }
    }
}

