package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name="ArmLogOpmode", group="Iterative Opmode")
public class ArmLoggingOp extends OpMode {
    public Servo myServo;
    @Override
    public void init() {
        setOpMode(this);
        //ServoNameInConfig is whatever you set the name of the servo to in config
        myServo = hardwareMap.get(Servo.class, "ServoNameInConfig");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //Tune this value
        myServo.setPosition(1);
    }
    @Override
    public void loop() {
        //Tune this value too
        myServo.setPosition(0);
    }
    @Override
    public void stop(){

        /*
                    Y O U R   C O D E   H E R E
                                                   */

    }
}
