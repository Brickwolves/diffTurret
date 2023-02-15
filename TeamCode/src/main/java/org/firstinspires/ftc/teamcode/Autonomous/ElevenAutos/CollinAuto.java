package org.firstinspires.ftc.teamcode.Autonomous.ElevenAutos;

import static org.firstinspires.ftc.teamcode.Autonomous.CollinTune.MoveTune.x;
import static org.firstinspires.ftc.teamcode.Autonomous.CollinTune.MoveTune.y;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.proportionalWeight;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.CollinMovement;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.opencv.core.Point;


@Autonomous(name="CollinAuto", group="Autonomous Linear Opmode")
public class CollinAuto extends LinearOpMode {

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

            movement.driveToPoint(5, 10, 0);
            multTelemetry.update();

        }
    }
}
