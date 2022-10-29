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


   public Robot() {
      initRobot();
   }

   public void initRobot() {

      drivetrain = new LupineMecanumDrive(hardwareMap);
      grabber = new Grabber();

      gyro = new IMU("imu");


      multTelemetry.addData("Status", "Initialized");
      multTelemetry.update();
   }



}