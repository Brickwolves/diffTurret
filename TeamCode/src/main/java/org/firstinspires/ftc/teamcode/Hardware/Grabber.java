package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.close;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.coneBlue;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.coneRed;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.depositDrop;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.groundJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.highJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.lowJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.midJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.open;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.Color_Sensor;
import org.firstinspires.ftc.teamcode.Utilities.Loggers.Side;

public class Grabber {


//    public CRServo grab1;
//    public CRServo grab2;
    public DcMotor spool;
    public DcMotor spool2;
    public Servo squeezer;
//    public Color_Sensor grabColor;

    public boolean wentDown = false;

    public ElapsedTime time = new ElapsedTime();
//    public Color_Sensor grabColor1;
//    public Color_Sensor grabColor2;
//    public DistanceSensor grabDistance;


    public Grabber(){
//        grab1 = hardwareMap.get(CRServo.class, "grab1");
//        grab2 = hardwareMap.get(CRServo.class, "grab2");

        squeezer = hardwareMap.get(Servo.class, "squeeze");
        squeezer.setPosition(0);

//        grabColor = new Color_Sensor();
//        grabColor.init("grabColor");

        spool = hardwareMap.get(DcMotor.class, "spool");
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setTargetPosition(0);
        spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spool.setPower(.2);

        spool2 = hardwareMap.get(DcMotor.class, "spool2");
        spool2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool2.setTargetPosition(0);
        spool2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spool2.setPower(.2);

//        grabColor1 = new Color_Sensor();
//        grabColor2 = new Color_Sensor();

    }

//    public boolean isLoaded(){
//        if(Side.red) {
//            return grabColor.updateRed() > coneRed;
//        }else{
//            return grabColor.updateBlue() > coneBlue;
//        }
//    }
//
//    public void intake(){
//        squeezer.setPosition(.1);
//
//        if(!isLoaded()) {
//            grab1.setPower(-intakeSpeed);
//            grab2.setPower(intakeSpeed);
//        }else{
//            stopIntake();
//        }
//    }
//
//    public void stopIntake(){
//        grab1.setPower(0);
//        grab2.setPower(0);
//    }
//
//    public void runIntakeBackwards(){
//        grab1.setPower(1);
//        grab2.setPower(-1);
//    }

    public void open(){
        squeezer.setPosition(open);
    }
    public void close(){
        squeezer.setPosition(close);
    }

    public void high(){
        spool.setPower(.3);
        spool2.setPower(.3);
        spool.setTargetPosition(highJunction);
        spool2.setTargetPosition(-highJunction);
    }

    public void middle(){
        spool.setPower(.3);
        spool.setTargetPosition(midJunction);
        spool2.setPower(.3);
        spool2.setTargetPosition(-midJunction);

    }

    public void low(){
        spool.setPower(.3);
        spool.setTargetPosition(lowJunction);
        spool2.setPower(.3);
        spool2.setTargetPosition(-lowJunction);
    }

    public void ground(){
        spool.setPower(.3);
        spool.setTargetPosition(groundJunction);
        spool2.setPower(.3);
        spool2.setTargetPosition(-groundJunction);
    }

    public void deposit() {
        spool.setPower(.3);
        spool2.setPower(.3);
        if(!wentDown) {
            spool.setTargetPosition((spool.getCurrentPosition() - depositDrop));
            spool2.setTargetPosition(-(spool.getCurrentPosition() - depositDrop));
            wentDown = true;
        }
        if(time.seconds() > .6) {
            squeezer.setPosition(.5);
        }
    }

    public void down(){
        spool.setPower(.4);
        spool.setTargetPosition(0);
        spool2.setPower(.4);
        spool2.setTargetPosition(0);
    }





}