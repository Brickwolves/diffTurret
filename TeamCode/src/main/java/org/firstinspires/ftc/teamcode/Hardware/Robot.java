package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;

/**
 * A class for containing an FTC Mecanum robot
 */
public class Robot {

   public LupineMecanumDrive drivetrain;
   public IMU gyro;
   public Grabber grabber;
   public Scoring scorer;



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
   }



}