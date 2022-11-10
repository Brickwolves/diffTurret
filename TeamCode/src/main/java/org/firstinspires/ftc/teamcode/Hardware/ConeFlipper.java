package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.flipDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.flipUp;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class ConeFlipper {

    public Servo flipper;

    public ConeFlipper(){
        flipper = hardwareMap.get(Servo.class, "flip");
    }

    public void up(){
        flipper.setPosition(flipUp);
    }
    public void down(){
        flipper.setPosition(flipDown);
    }

}
