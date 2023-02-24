package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

      //Trajectories for Left Cycles
      startLeft = new Pose2d(30,60,Math.toRadians(90));

      postRotateLeft = new Pose2d(50,15,Math.toRadians(150));

      lowCycleLeft2 = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
              .lineToConstantHeading(new Vector2d(58,15))
              .build();
      lowCycleLeft3 = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
              .lineToConstantHeading(new Vector2d(50,23))
              .build();
      lowCycleLeft4 = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
              .lineToConstantHeading(new Vector2d(58,15))
              .build();
      park1Left = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
              .lineToConstantHeading(new Vector2d(11,12))
              .build();
      park2Left = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
              .lineToConstantHeading(new Vector2d(35,12))
              .build();

      //Trajectories for Right Cycles
      startRight = new Pose2d()
   }


   public void cycleLowLeft(int cycles){
      int stackHeight = 5;
      drivetrain.turnTo(Math.toRadians(150));
      //stack position
      drivetrain.followTrajectory(lowCycleLeft2);
      while(cycles > 0){
         //close grabber
         //escape from stack with stack height of stackHeight
         stackHeight = stackHeight - 1;
         //v4b to low goal position
         drivetrain.followTrajectory(lowCycleLeft3);
         //deposit
         if(cycles == 1){
            break;
         }
         //stack position
         drivetrain.followTrajectory(lowCycleLeft4);
         cycles = cycles - 1;
      }
   }




}