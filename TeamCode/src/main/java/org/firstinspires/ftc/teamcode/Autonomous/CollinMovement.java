package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Autonomous.CollinMovement.CollinMoveDash.*;
import static org.firstinspires.ftc.teamcode.Autonomous.CollinTune.MoveTune.x;
import static org.firstinspires.ftc.teamcode.Autonomous.CollinTune.MoveTune.y;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.proportionalWeight;

import static java.lang.Math.hypot;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.opencv.core.Point;

public class CollinMovement {

    @Config
    public static class CollinMoveDash{
        public static double driveP = 0;
        public static double driveD = 0;
        public static double driveF = 0;

        public static double strafeP = 0;
        public static double strafeD = 0;
        public static double strafeF = 0;
    }

    private PID drivePID;
    private PID strafePID;
    private PID rotationPID;
    private Robot robot;

    public CollinMovement(Robot robot){
        this.robot = robot;
        drivePID = new PID(driveP, 0, driveD);
        strafePID = new PID(strafeP, 0, strafeD);
        rotationPID = new PID(proportionalWeight, integralWeight, derivativeWeight);
    }

    public Point getRelInputs(Point targetPosition, Pose2d robotPosition, double robotRotation){
        drivePID.setWeights(driveP, 0, driveD);
        strafePID.setWeights(strafeP, 0, strafeD);
        Point error = new Point(targetPosition.x - robotPosition.getX(), targetPosition.y - robotPosition.getY());
        error = MathUtils.shift(error, 360 - robotRotation);
        return new Point(
                drivePID.update(error.x, false) + driveF*signum(error.x),
                strafePID.update(error.y, false) + strafeF*signum(error.y)

        );
    }

    public double tuneStrafe(double error){
        strafePID.setWeights(strafeP, 0, strafeD);
        return strafePID.update(error, false) + strafeF*signum(error);
    }

    public double tuneDrive(double error){
        drivePID.setWeights(driveP, 0, driveD);
        return drivePID.update(error, false) + driveF*signum(error);
    }

    public double driveToPoint(Point targetPoint, double rotationHold){
        rotationPID.setWeights(proportionalWeight, integralWeight, derivativeWeight);

        robot.drivetrain.update();
        robot.drivetrain.getPoseEstimate();

        Point correction = getRelInputs(new Point(x,y), robot.drivetrain.getPoseEstimate(), robot.gyro.getAngle());

        robot.drivetrain.setDrivePower(
                correction.x,
                correction.y,
                rotationPID.update(robot.gyro.getAngle() - 0, true), 1);

        multTelemetry.addData("x", robot.drivetrain.getPoseEstimate().getX());
        multTelemetry.addData("y", robot.drivetrain.getPoseEstimate().getY());
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setStrokeWidth(1)
                .setFill("black").fillCircle(robot.drivetrain.getPoseEstimate().getX(), robot.drivetrain.getPoseEstimate().getY(), 2)
                .setFill("red").fillCircle(targetPoint.x, targetPoint.y, 1.5);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        return hypot(targetPoint.x - robot.drivetrain.getPoseEstimate().getX(), targetPoint.y - robot.drivetrain.getPoseEstimate().getY());
    }

    public double driveToPoint(double x, double y, double rotationHold){
        return driveToPoint(new Point(x, y), rotationHold);
    }


}
