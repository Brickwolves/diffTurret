package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    DcMotor intake;
    public double resistance = 0;
    public int iteration;
    public double lastPos = 0;
    public double currentPos;
    public double speed;
    public Intake(){
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public void runIntake(double speed){
        intake.setPower(speed);
    }

    public double updateSpeed(){
        iteration++;
        if(iteration == 20){
           iteration = 0;
           currentPos = intake.getCurrentPosition();
           speed = Math.abs(currentPos - lastPos);
           lastPos = currentPos;
        }
        return speed;
    }

    public double getSpeed(){
        return speed;
    }

    public double getResistance(){
        return 1;
    }
}
