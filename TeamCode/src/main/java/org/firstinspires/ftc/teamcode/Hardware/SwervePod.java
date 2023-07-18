package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.AbsoluteEncoder;

public class SwervePod {
    DcMotor roll;
    CRServo yaw;
    AbsoluteEncoder encoder;


    public double setAngle(double angle){

        return encoder.getAngle();
    }

    public void setPowers(){

    }
}
