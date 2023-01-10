package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawClose;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawOpen;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawTipped;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberTip;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.highFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.highJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.midFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.midJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.tipAngle;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.tippedHeight;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreBack;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreFrontLow;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.inRange;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Scoring {

    public DcMotor spool;
    public DcMotor spool2;
    public Servo squeezer;
    public Servo v4b1;
    public Servo v4b2;
    public Servo grabberSpin;


    public boolean wentDown = false;

    public ElapsedTime time = new ElapsedTime();



    public Scoring(){

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

    //Base Movement Methods
    public void open(boolean tipped){
        if(tipped){
            squeezer.setPosition(clawTipped);
        }else {
            squeezer.setPosition(clawOpen);
        }
    }
    public void close(){
        squeezer.setPosition(clawClose);
    }
    public void v4b(double target){
        v4b1.setPosition(target);
        v4b2.setPosition(1-target);
    }
    public void grabber(double target){
        grabberSpin.setPosition(target);
    }
    public void slides(double power, int target){
        spool.setPower(power);
        spool2.setPower(power);
        spool.setTargetPosition(-target);
        spool2.setTargetPosition(target);
    }

    //Score Methods

    //Front Score
    public void highFront(boolean funny) {
        if (!funny) {
            close();
            slides(1, highJunction);
            v4b(v4bScoreFront);
            if (inRange(.2, time.seconds(), 1)) {
                grabber(grabberScoreFront);
            }
        }else{
            highBack(true);
        }
    }

    public void midFront(boolean funny){
        if (!funny) {
        close();
        slides(1,midJunction);
        v4b(v4bScoreFront);
        if(inRange(.2, time.seconds(), 1)){
            grabber(grabberScoreFront);
        }
        }else{
            highBack(true);
        }
    }

    public void lowFront(boolean funny){
        if (!funny) {
        close();
        slides(1,0);
        v4b(v4bScoreFrontLow);
        if(inRange(.2, time.seconds(), 1)){
            grabber(grabberScoreFront);
        }
        }else{
            highBack(true);
        }
    }

    //Back Score
    public void highBack(boolean funny){
        close();
        v4b(v4bScoreBack);
        if(inRange(.2, time.seconds(), 1)){
            if(funny) {
                slides(1,highFunny);
                grabber(grabberFunny);
            }else{
                grabber(grabberScore);
                slides(1,highJunction);
            }
        }
    }

    public void midBack(boolean funny){
        close();
        v4b(v4bScoreBack);
        if(inRange(.2, time.seconds(), 1)){
            if(funny) {
                slides(1,midFunny);
                grabber(grabberFunny);
            }else{
                grabber(grabberScore);
                slides(1,midJunction);
            }
        }
    }

    public void lowBack(boolean funny){
        close();
        slides(1,0);
        v4b(v4bScoreBack);
        if(inRange(.2, time.seconds(), 1)){
            if(funny) {
                grabber(grabberFunny);
            }else{
                grabber(grabberScore);
            }
        }
    }

    //Deposit
    public void deposit(String coneAngle){
        if(inRange(0,time.seconds(), .3)){
            open(false);
        }
        if(inRange(.3,time.seconds(), 2)){
            close();
        }
        if(inRange(.5,time.seconds(),2)){
            v4b(v4bDown);
        }
        if(time.seconds()>.8) {
            if(coneAngle.equals("Straight")) {
                grabber(grabberDown);
                slides(1,0);
            } else if(coneAngle.equals("Forwards")) {
                grabber(grabberTip);
                slides(.5,tippedHeight);
            }
        }
    }




    //Immediately Drop Slides
    public void crashSlides(){
        close();
        slides(1,0);
        v4b(v4bDown);
    }
}
