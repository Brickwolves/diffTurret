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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Vision.SignalPipeline;

@Autonomous(name="Test Auto", group="Autonomous Linear Opmode")
public class TestAuto extends LinearOpMode {
    Robot robot;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public SignalPipeline.SignalSide signalSide;


    public void initialize() {
        setOpMode(this);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {


        initialize();

        robot = new Robot(true);


        //To change speed, pass regulateSpeed1(*whateverspeedyouwant*) as an argument of Pose2D, followed by regulateSpeed2()






        waitForStart();

        if (opModeIsActive()) {

            runtime.reset();
            while(runtime.seconds()<20) {
                robot.scorer.autoHigh();
            }


            telemetry.addData("finalX", robot.drivetrain.getPoseEstimate().getX());
            telemetry.addData("finalY", robot.drivetrain.getPoseEstimate().getY());
            telemetry.update();
        }

    }

}

