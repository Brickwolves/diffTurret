package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawClose;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.depositDrop;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.highJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.midJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawOpen;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Grabber {


//    public CRServo grab1;
//    public CRServo grab2;
    public DcMotor spool;
    public DcMotor spool2;
    public Servo squeezer;
    public Servo v4b1;
    public Servo v4b2;
    public Servo grabberSpin;
//    public Color_Sensor grabColor;

    public boolean wentDown = false;

    public ElapsedTime time = new ElapsedTime();


    public Grabber(){

        squeezer = hardwareMap.get(Servo.class, "squeeze");
        squeezer.setPosition(.3);

        v4b1 = hardwareMap.get(Servo.class, "v4b1");
        v4b2 = hardwareMap.get(Servo.class,"v4b2");

        grabberSpin = hardwareMap.get(Servo.class, "spin");


        spool = hardwareMap.get(DcMotor.class, "spool");
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setTargetPosition(0);
        spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spool.setPower(.6);

        spool2 = hardwareMap.get(DcMotor.class, "spool2");
        spool2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool2.setTargetPosition(0);
        spool2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spool2.setPower(.6);

    }



    public void open(){
        squeezer.setPosition(clawOpen);
    }
    public void close(){
        squeezer.setPosition(clawClose);
    }


    //2 is set to highest at 0
    //1 is set to highest at 1
    public void rotate(double target){
        v4b1.setPosition(target);
        v4b2.setPosition(1-target);
    }

    public void rotateGrabber(double target){
        grabberSpin.setPosition(target);
    }

    public void high(){
        spool.setPower(1);
        spool2.setPower(1);
        spool.setTargetPosition(-highJunction);
        spool2.setTargetPosition(highJunction);
    }

    public void middle(){
        spool.setPower(.8);
        spool.setTargetPosition(-midJunction);
        spool2.setPower(.8);
        spool2.setTargetPosition(midJunction);

    }

    public void low(){
        spool.setPower(.6);
        spool.setTargetPosition(0);
        spool2.setPower(.6);
        spool2.setTargetPosition(0);
    }

    public void ground(){
        spool.setPower(.3);
        spool.setTargetPosition(0);
        spool2.setPower(.3);
        spool2.setTargetPosition(0);
    }

    public void deposit() {
        spool.setPower(.3);
        spool2.setPower(.3);
        if(!wentDown) {
            spool.setTargetPosition(-(spool.getCurrentPosition() - depositDrop));
            spool2.setTargetPosition((spool.getCurrentPosition() - depositDrop));
            wentDown = true;
        }
        if(time.seconds() > .6) {
            squeezer.setPosition(.5);
        }
    }

    public void down(){
        spool.setPower(1);
        spool.setTargetPosition(0);
        spool2.setPower(1);
        spool2.setTargetPosition(0);
    }





}