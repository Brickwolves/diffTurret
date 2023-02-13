package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawClose;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawOpen;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.clawTipped;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberHide;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberStartAuto;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberTip;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.highFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.highFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.highJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.midFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.midFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.midJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.stackIncrease;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.stackedHeight;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.tipAngle;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.tippedHeight;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreBack;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreFrontLow;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bStartAuto;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffset;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.inRange;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.NonConstants.fullyDown;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.vD;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.vI;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.vP;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        slides(1,midJunction);
        sleep(0.4);
        v4b(v4bScoreBack);
        grabber(grabberScore);

    }

    public void autoLow(){

    }

    public void autoDeposit(){
        open(false);
        sleep(0.4);
        close();
        v4b(v4bDown);
        sleep(0.6);
        slides(1,0);
        autoStart();

    }

    public void stackPickup(int height){
        v4b(v4bDown);
        slides(1,(height-1)*stackIncrease);
    }

    public void stackEscape(int height){
        slides(1,(height+4)*stackIncrease);
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
        //1 should be changed to absolute encoder value
        v4b1.setPower(v4bPID.update(target - 1, false));
        v4b2.setPower(1-(v4bPID.update(target - 1, false)));
    }

    public void v4bNoSensor(double speed){
        v4b1.setPower(speed);
        v4b2.setPower(-speed);
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
            slides(1, highFront);
            v4b(v4bScoreFront);
            if (time.seconds() > .2) {
                v4b(v4bScoreFront);
                grabber(grabberScoreFront);
            }
        }else{
            highBack(true);
        }
    }

    public void midFront(boolean funny){
        if (!funny) {
            close();
            slides(1,midFront);
            v4b(v4bScoreFront);
            if(time.seconds() > .2){
                v4b(v4bScoreFront);
                grabber(grabberScoreFront);
            }
        }else{
            midBack(true);
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
            lowBack(true);
        }
    }

    //Back Score
    public void highBack(boolean funny){
        close();
        v4b(v4bScoreBack);
        if(time.seconds() > .2){
            v4b(v4bScoreBack);
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
        if(time.seconds() > .2){
            v4b(v4bScoreBack);
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
        if(time.seconds() > .2){
            v4b(v4bScoreBack);
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
            fullyDown = false;
        }
        if(inRange(.3,time.seconds(), .5)){
            close();
        }
        if(inRange(.7,time.seconds(),.8)){
            v4b(v4bDown);
            slides(1,0);
            grabber(grabberHide);
        }
        if(inRange(1.5,time.seconds(),1.7)){
            if(coneAngle.equals("Straight")) {
                open(false);
            } else if(coneAngle.equals("Forwards")) {
                open(true);
            }
        }
        if(1.5<time.seconds()) {
            if(coneAngle.equals("Straight")) {
                grabber(grabberDown);
                slides(1,0);
            } else if(coneAngle.equals("Forwards")) {
                grabber(grabberTip);
                slides(.5,tippedHeight);
            }
        }
        if(time.seconds()>1.8){
            fullyDown = true;
            v4b(v4bDown);
        }
    }

    public void deposit(String coneAngle, boolean start){
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
        return (Math.abs(spool.getCurrentPosition()) + Math.abs(spool2.getCurrentPosition()))/2;
    }
}
