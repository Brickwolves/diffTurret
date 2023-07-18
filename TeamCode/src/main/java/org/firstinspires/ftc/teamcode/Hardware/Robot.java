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
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Distance_Sensor;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Odometry.drive.StandardTrackingWheelLocalizer;

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
   public Distance_Sensor distance;
//   public StandardTrackingWheelLocalizer localizer;






   public Robot() {
      initRobot();
      grabber = new Grabber();
   }
   public Robot(boolean advancedScoring){
      initRobot();
      scorer = new Scoring();
      distance = new Distance_Sensor();
      distance.init("distance");
   }


   public void initRobot() {

      drivetrain = new LupineMecanumDrive(hardwareMap);
      gyro = new IMU("imu");
//      localizer = new StandardTrackingWheelLocalizer(hardwareMap);


   }
}