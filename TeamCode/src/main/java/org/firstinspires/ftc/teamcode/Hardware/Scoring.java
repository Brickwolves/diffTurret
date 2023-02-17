package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawClose;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawOpen;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawOpenScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawTipped;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberScoreFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberHide;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberScoreFunnyFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberStartAuto;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberTip;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.slidesHighFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.slidesHighFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.slidesHighJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.slidesMidFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.slidesMidFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.slidesMidJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.slidesStackIncrease;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.slidesTippedHeight;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bDownFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreBack;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreBackLow;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreFrontLow;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bStartAuto;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bUndershoot;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bUndershootSpeed;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffset;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.inRange;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.NonConstants.fullyDown;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.vD;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.vI;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.vP;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.Utilities.PID;

public class Scoring {

    public DcMotor spool;
    public DcMotor spool2;
    public Servo squeezer;
    public CRServo v4b1;
    public CRServo v4b2;
    public PID v4bPID;
    public Servo grabberSpin;
    public TouchSensor beam1;
    public Intake intake;
    public AbsoluteEncoder encoder;
    public boolean previousPress = false;
    public boolean clawToggleOpen = false;


    public boolean wentDown = false;

    public ElapsedTime time = new ElapsedTime();
    public ElapsedTime sleep = new ElapsedTime();



    public Scoring(){

        intake = new Intake();

        squeezer = hardwareMap.get(Servo.class, "squeeze");
        squeezer.setPosition(.3);

        v4b1 = hardwareMap.get(CRServo.class, "v4b1");
        v4b2 = hardwareMap.get(CRServo.class,"v4b2");
        v4bPID = new PID(vP,vI,vD);

        grabberSpin = hardwareMap.get(Servo.class, "spin");

        encoder = new AbsoluteEncoder("encoder");

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

    }

    public void autoMid(){
        sleep(0.4);
        slides(1, slidesMidJunction);
        sleep(0.4);
        v4b(v4bScoreBack);
        grabber(grabberScore);

    }

    public void autoLow(){
        sleep(0.4);
        slides(1, 0);
        v4b(v4bScoreBackLow);
        grabber(grabberScore);

    }

    public void autoDeposit(){
        openScore();
        sleep(0.4);
        close();
        v4b(v4bDown);
        sleep(0.6);
        slides(1,0);
        autoStart();

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

    public void v4b(double target){
        //if velocity is more undershoot velocity, target becomes undershoot of target
        //basically if you're fast because far away, go for undershoot, but when you get close, go to your actual position
        double velocity = encoder.getVelocity();
        double adjustedTarget = abs(velocity) > v4bUndershootSpeed ?
                target + (v4bUndershoot * -Math.signum(target - encoder.getAngle()))
                : target;
        v4bPID.setWeights(vP,vI,vD);
        if(inRange(adjustedTarget-2, encoder.getAngle(), adjustedTarget+2)) {
            v4b1.setPower(0);
            v4b2.setPower(0);
        }else {
            v4b1.setPower(v4bPID.update(adjustedTarget - encoder.getAngle(), false));
            v4b2.setPower(-(v4bPID.update(adjustedTarget - encoder.getAngle(), false)));
        }
        multTelemetry.addData("adjusted target", adjustedTarget);
        multTelemetry.addData("get angle", encoder.getAngle());

    }

    public void v4bNoSensor(double speed){
        v4b1.setPower(speed);
        v4b2.setPower(-speed);
    }

    public void v4bNoPID(double target){
        if(!inRange(target-2,encoder.getAngle(),target+2)){
            if(encoder.getAngle() > target){
                v4b1.setPower(-1);
                v4b2.setPower(1);
            }else if(encoder.getAngle() < target){
                v4b1.setPower(1);
                v4b2.setPower(-1);
            }
        }
    }

    public void grabber(double target){
        grabberSpin.setPosition(target);
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
