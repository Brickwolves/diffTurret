package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;

/**
 * A class for containing an FTC Mecanum robot
 */
public class Robot {

   public LupineMecanumDrive drivetrain;
   public IMU gyro;
   public Grabber grabber;
   public Scoring scorer;
   public Trajectory midPreloadLeft1;
   public Trajectory midPreloadLeft2;
   public Trajectory lowCycleLeft1;
   public Trajectory lowCycleLeft2;
   public Trajectory lowCycleLeft3;
   public Trajectory lowCycleLeft4;
   public Pose2d startLeft;
   public Pose2d postRotateLeft;
   public Trajectory park1;
   public Trajectory park2;






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


      multTelemetry.addData("Status", "Initialized");
      multTelemetry.update();
      startLeft = new Pose2d(30,60,Math.toRadians(90));

      midPreloadLeft1 = drivetrain.trajectoryBuilder(startLeft)
              .lineToConstantHeading(new Vector2d(35,45))
              .build();

      midPreloadLeft2 = drivetrain.trajectoryBuilder(midPreloadLeft1.end())
              .lineToConstantHeading(new Vector2d(26,27))
              .build();
      lowCycleLeft1 = drivetrain.trajectoryBuilder(midPreloadLeft2.end())
              .lineToConstantHeading(new Vector2d(50,15))
              .build();
      postRotateLeft = new Pose2d(50,15,Math.toRadians(150));
      lowCycleLeft2 = drivetrain.trajectoryBuilder(postRotateLeft)
              .lineToConstantHeading(new Vector2d(58,15))
              .build();
      lowCycleLeft3 = drivetrain.trajectoryBuilder(lowCycleLeft2.end())
              .lineToConstantHeading(new Vector2d(50,23))
              .build();
      lowCycleLeft4 = drivetrain.trajectoryBuilder(lowCycleLeft3.end())
              .lineToConstantHeading(new Vector2d(58,15))
              .build();
      park1 = drivetrain.trajectoryBuilder(lowCycleLeft3.end())
              .lineToConstantHeading(new Vector2d(11,12))
              .build();
      park2 = drivetrain.trajectoryBuilder(lowCycleLeft3.end())
              .lineToConstantHeading(new Vector2d(35,12))
              .build();
   }
   public void preloadMidLeft(){
      drivetrain.followTrajectory(midPreloadLeft1);
      drivetrain.followTrajectory(midPreloadLeft2);
      drivetrain.turnTo(Math.toRadians(45));
      scorer.autoMid();
      scorer.sleep(1);
      scorer.autoDeposit();
   }

   public void cycleLowLeft(int cycles){
      int stackHeight = 5;
      drivetrain.followTrajectory(lowCycleLeft1);
      drivetrain.turnTo(150);
      scorer.stackPickup(stackHeight);
      drivetrain.followTrajectory(lowCycleLeft2);
      while(cycles > 0){
         scorer.close();
         scorer.stackEscape(stackHeight);
         stackHeight = stackHeight - 1;
         scorer.autoLow();
         drivetrain.followTrajectory(lowCycleLeft3);
         scorer.autoDeposit();
         if(cycles == 1){
            break;
         }
         scorer.stackPickup(stackHeight);
         drivetrain.followTrajectory(lowCycleLeft4);
         cycles = cycles - 1;
      }
   }




}