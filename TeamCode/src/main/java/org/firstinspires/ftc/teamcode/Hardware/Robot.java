package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.ar.core.Pose;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;

/**
 * A class for containing an FTC Mecanum robot
 */
public class Robot {

   public LupineMecanumDrive drivetrain;
   public IMU gyro;
   public Grabber grabber;
   public Scoring scorer;
   public Trajectory lowCycleLeft2;
   public Trajectory lowCycleLeft3;
   public Trajectory lowCycleLeft4;
   public Pose2d startLeft;
   public Pose2d postRotateLeft;
   public Trajectory park1Left;
   public Trajectory park2Left;
   public Pose2d startRight;
   public Localizer localizer;






   public Robot() {
      initRobot();
      grabber = new Grabber();
   }
   public Robot(boolean advancedScoring){
      initRobot();
      scorer = new Scoring();

   }


   public void initRobot() {

      drivetrain = new LupineMecanumDrive(hardwareMap);
      gyro = new IMU("imu");
      localizer = new Localizer() {
         @NonNull
         @Override
         public Pose2d getPoseEstimate() {
            return null;
         }

         @Override
         public void setPoseEstimate(@NonNull Pose2d pose2d) {

         }

         @Nullable
         @Override
         public Pose2d getPoseVelocity() {
            return null;
         }

         @Override
         public void update() {

         }
      };


      multTelemetry.addData("Status", "Initialized");
      multTelemetry.update();

      //Trajectories for Left Cycles
      startLeft = new Pose2d(30,60,Math.toRadians(90));

      postRotateLeft = new Pose2d(45,13,Math.toRadians(150));

      lowCycleLeft2 = drivetrain.trajectoryBuilder(postRotateLeft)
              .lineToConstantHeading(new Vector2d(60,17))
              .build();
      lowCycleLeft3 = drivetrain.trajectoryBuilder(lowCycleLeft2.end())
              .lineToConstantHeading(new Vector2d(52,20))
              .build();
      lowCycleLeft4 = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
              .lineToConstantHeading(new Vector2d(60,17))
              .build();
      park1Left = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
              .lineToConstantHeading(new Vector2d(11,12))
              .build();
      park2Left = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
              .lineToConstantHeading(new Vector2d(35,12))
              .build();

      //Trajectories for Right Cycles
      startRight = new Pose2d();
   }


   public void cycleLowLeft(int cycles){
      int stackHeight = 5;
      drivetrain.turnTo(Math.toRadians(180));
      scorer.stackPickup(stackHeight);
      drivetrain.setPoseEstimate(drivetrain.getPoseEstimate());
      drivetrain.followTrajectory(lowCycleLeft2);
      while(cycles > 0){
         while (!scorer.beamBroken()){
            drivetrain.setDrivePower(1,0,0,.2);
         }
         drivetrain.setDrivePower(1,0,0,0);
         scorer.close();
         scorer.stackEscape(stackHeight);
         stackHeight = stackHeight - 1;
         scorer.autoLow();
         drivetrain.setPoseEstimate(drivetrain.getPoseEstimate());
         drivetrain.followTrajectory(lowCycleLeft3);
         drivetrain.turnTo(Math.toRadians(270));
         scorer.autoDeposit();
         cycles = cycles - 1;
         if (cycles == 0){
            break;
         }
         drivetrain.setPoseEstimate(drivetrain.getPoseEstimate());
         drivetrain.followTrajectory(lowCycleLeft2);
         }
   }




}