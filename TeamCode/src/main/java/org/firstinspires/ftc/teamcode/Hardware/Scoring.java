package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4b0;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4b90;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bSpeed;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawClose;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawOpen;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawOpenScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawTipped;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabber0;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabber90;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberScoreFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberHide;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberScoreFunnyFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberStartAuto;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberTip;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesHighFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesHighFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesHighJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesMidFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesMidFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesMidJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesStackIncrease;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesTippedHeight;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bDownFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bScoreBack;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bScoreBackLow;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bScoreFrontLow;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bStartAuto;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffset;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.inRange;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.interpolateRanges;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.NonConstants.fullyDown;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.vD;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.vI;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.vP;

import static java.lang.Math.abs;
import static java.lang.Math.round;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.PID;

public class Scoring {

    public DcMotor spool;
    public DcMotor spool2;
    public Servo squeezer;
    public DcMotor v4b;
    public PID v4bPID;
    public Servo grabberSpin;
    public TouchSensor beam1;
    public Intake intake;
    public boolean previousPress = false;
    public boolean clawToggleOpen = false;


    public boolean wentDown = false;

    public ElapsedTime time = new ElapsedTime();
    public ElapsedTime sleep = new ElapsedTime();



    public Scoring(){

        intake = new Intake();

        squeezer = hardwareMap.get(Servo.class, "squeeze");
        v4b = hardwareMap.get(DcMotor.class,"v4b");
        v4b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        v4b.setTargetPosition(0);
        v4b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        v4b.setPower(.5);

        v4bPID = new PID(vP,vI,vD);

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

        beam1 = hardwareMap.get(TouchSensor.class, "beam1");
    }

    //AUTO
    public void autoHigh(){
        v4b(v4bScoreBack);
    }

    public void autoMid(){
        sleep(0.4);
        slides(1, slidesMidJunction);
        grabber(grabberScore);
        sleep(0.4);
        v4b(v4bScoreBack);

    }

    public void autoLow(){
        sleep(0.4);
        slides(1, 0);
        v4b(v4bScoreBackLow);
        grabber(grabberScore);

    }

    public void autoDeposit(){
        openScore();
        sleep(0.8);
        close();
        v4b(v4bDown);
        sleep(0.6);
        slides(1,0);

    }

    public void stackPickup(int height){
        v4b(v4bDown);
        slides(1,(height-1)* slidesStackIncrease);
    }

    public void stackEscape(int height){
        slides(1,(height+4)* slidesStackIncrease);
    }

    public boolean beamBroken(){
        return beam1.isPressed();
    }

    public boolean updateBeam(){
        boolean retVal = beam1.isPressed() && !previousPress;
        previousPress = beam1.isPressed();
        return retVal;
    }


    //Base Movement Methods
    public void open(boolean tipped){
        if(tipped){
            squeezer.setPosition(clawTipped);
        }else {
            squeezer.setPosition(clawOpen);
        }
    }

     public void openScore(){
        squeezer.setPosition(clawOpenScore);
     }
    public void close(boolean down){
        if(down){
            slides(1,100);
        }
        squeezer.setPosition(clawClose);
    }

    public void close(){
        squeezer.setPosition(clawClose);
    }


    public void v4b(int target) {
        v4b.setPower(v4bSpeed);
        v4b.setTargetPosition((int) interpolateRanges(target, 0, 90, v4b0, v4b90));
        multTelemetry.addData("target", target);
        multTelemetry.addData("current", v4b.getCurrentPosition());
    }

    public void grabber(double target){
        grabberSpin.setPosition(target);
        //grabberSpin.setPosition(interpolateRanges(target, 0, 90, grabber0, grabber90));
    }

    public void slides(double power, int target){
        int newTarget = target - (int)slidesOffset;
        spool.setPower(power);
        spool2.setPower(power);
        spool.setTargetPosition(-newTarget);
        spool2.setTargetPosition(newTarget);
    }

    //Score Methods

    //Front Score
    public void highFront(boolean funny) {
        if (!funny) {
            close();
            slides(1, slidesHighFront);
            v4b(v4bScoreFront);
            if (time.seconds() > .2) {
                v4b(v4bScoreFront);
                grabber(grabberScoreFront);
            }
        }else{
            close();
            slides(1, slidesHighFront);
            v4b(v4bScoreFront);
            if (time.seconds() > .2) {
                v4b(v4bScoreFront);
                grabber(grabberScoreFunnyFront);
            }
        }
    }

    public void midFront(boolean funny){
        if (!funny) {
            close();
            slides(1, slidesMidFront);
            v4b(v4bScoreFront);
            if(time.seconds() > .2){
                v4b(v4bScoreFront);
                grabber(grabberScoreFront);
            }
        }else{
            close();
            slides(1, slidesMidFront);
            v4b(v4bScoreFront);
            if(time.seconds() > .2){
                v4b(v4bScoreFront);
                grabber(grabberScoreFunnyFront);
            }
        }
    }

    public void lowFront(boolean funny){
        if (!funny) {
            close();
            slides(1,0);
            v4b(v4bScoreFrontLow);
            if(time.seconds() > .2){
                v4b(v4bScoreFrontLow);
                grabber(grabberScoreFront);
            }
        }else{
            close();
            slides(1,0);
            v4b(v4bScoreFrontLow);
            if(time.seconds() > .2){
                v4b(v4bScoreFrontLow);
                grabber(grabberScoreFunnyFront);
            }
        }
    }

    //Back Score
    public void highBack(boolean funny){
        close();
        v4b(v4bScoreBack);
        if(time.seconds() > .2){
            v4b(v4bScoreBack);
            if(funny) {
                slides(1, slidesHighFunny);
                grabber(grabberScoreFunny);
            }else{
                grabber(grabberScore);
                slides(1, slidesHighJunction);
            }
        }
    }

    public void midBack(boolean funny){
        close();
        v4b(v4bScoreBack);
        if(time.seconds() > .2){
            v4b(v4bScoreBack);
            if(funny) {
                slides(1, slidesMidFunny);
                grabber(grabberScoreFunny);
            }else{
                grabber(grabberScore);
                slides(1, slidesMidJunction);
            }
        }
    }

    public void lowBack(boolean funny){
        close();
        slides(1,0);
        v4b(v4bScoreBack);
        if(time.seconds() > .2){
            v4b(v4bScoreBack);
            if(funny) {
                grabber(grabberScoreFunny);
            }else{
                grabber(grabberScore);
            }
        }
    }

    //Deposit
    public void deposit(String coneAngle){
        if(inRange(0,time.seconds(), .3)){
            openScore();
            fullyDown = false;
        }
        if(inRange(.3,time.seconds(), .5)){
            close();
        }
        if(inRange(.7,time.seconds(),.8)){
            v4b(v4bDown);
            grabber(grabberHide);
        }
        if(inRange(1.5,time.seconds(),1.7)){
            slides(1,0);
            if(coneAngle.equals("Straight")) {
                open(false);
            } else if(coneAngle.equals("Forwards")) {
                v4b(v4bDownFunny);
                open(true);
            }
        }
        if(2<time.seconds()) {
            if(coneAngle.equals("Straight")) {
                grabber(grabberDown);
                slides(1,0);
            } else if(coneAngle.equals("Forwards")) {
                grabber(grabberTip);
                slides(.5, slidesTippedHeight);
            }
        }
        if(time.seconds()>1.8){
            fullyDown = true;
            if(coneAngle.equals("Forwards")){
                v4b(v4bDownFunny);
            }else {
                v4b(v4bDown);
            }
        }
    }

    public void startTeleop(String coneAngle){
        if(inRange(0,time.seconds(), .3)){
            open(false);
            fullyDown = true;
            grabber(grabberDown);
        }
        if(inRange(0.8,time.seconds(),2)){
            v4b(v4bDown);
        }
        if(time.seconds()>.8) {
            slides(1,0);
        }
    }



    //Immediately Drop Slides
    public void crashSlides(){
        close();
        slides(1,0);
        v4b(v4bDown);
    }

    public void autoStart(){
        close();
        grabber(grabberStartAuto);
        v4b(v4bStartAuto);
        slides(1,0);
    }

    public void sleep(double sleepTime){
        sleep.reset();
        while(sleep.seconds()<sleepTime){

        }
    }

    public int getHeight(){
        return (abs(spool.getCurrentPosition()) + abs(spool2.getCurrentPosition()))/2;
    }
}
