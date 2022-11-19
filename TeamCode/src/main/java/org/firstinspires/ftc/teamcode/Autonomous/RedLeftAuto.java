package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive.regulateSpeed1;
import static org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive.regulateSpeed2;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.VisionPipelines.SignalPipeline.SignalSide.ONE_GREEN;
import static org.firstinspires.ftc.teamcode.VisionPipelines.SignalPipeline.SignalSide.THREE_PINK;
import static org.firstinspires.ftc.teamcode.VisionPipelines.SignalPipeline.SignalSide.TWO_ORANGE;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Vision.Camera;
import org.firstinspires.ftc.teamcode.Utilities.Loggers.Side;
import org.firstinspires.ftc.teamcode.VisionPipelines.SignalPipeline;


@Autonomous(name="Blue Right Auto", group="Autonomous Linear Opmode")
public class RedLeftAuto extends LinearOpMode {
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
        camera = new Camera("junkCam", true);

        robot.drivetrain.setPoseEstimate(new Pose2d(-36,-66,Math.toRadians(90)));



        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()
        Trajectory traj1 = robot.drivetrain.trajectoryBuilder(new Pose2d(-30,-70,Math.toRadians(90)))
                .lineTo(new Vector2d(-34, 68),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .build();

        Trajectory traj2 = robot.drivetrain.trajectoryBuilder(new Pose2d(-30,-70,Math.toRadians(270)))
                .lineTo(new Vector2d(-50, 68),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .build();

        Trajectory traj3 = robot.drivetrain.trajectoryBuilder(new Pose2d(-30,-70,Math.toRadians(90)))
                .lineTo(new Vector2d(-70, 68),
                        regulateSpeed1(30),
                        regulateSpeed2())
                .build();

        multTelemetry.addData("signal side", signalSide);
        multTelemetry.update();


        waitForStart();

        if (opModeIsActive()) {

            if (signalSide == ONE_GREEN){
                robot.drivetrain.followTrajectory(traj1);
            }
            if (signalSide == TWO_ORANGE){
                robot.drivetrain.followTrajectory(traj2);
            }
            if (signalSide == THREE_PINK){
                robot.drivetrain.followTrajectory(traj3);
            }

        }
    }
}
