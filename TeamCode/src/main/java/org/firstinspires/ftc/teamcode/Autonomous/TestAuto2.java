package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.LupineMecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;


@Disabled
@Autonomous(name="Test Auto2", group = "Autonomous Linear Opmode")
public class TestAuto2 extends LinearOpMode {
    LupineMecanumDrive drive;

    public void initialize(){
        setOpMode(this);
    }

    @Override
    public void runOpMode(){
        initialize();
        drive = new LupineMecanumDrive(OpModeUtils.hardwareMap);
        Trajectory testTraj = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();

        waitForStart();

        if(opModeIsActive()){
            drive.followTrajectory(testTraj);
        }

    }

}
