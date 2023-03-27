package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;

/**
 * A class for containing an FTC Mecanum robot
 */
public class Robot {

   public Mecanum drivetrain;
   public IMU gyro;






   public Robot() {
      initRobot();
   }

   public void initRobot() {

      drivetrain = new Mecanum();
      gyro = new IMU("imu");


   }
}