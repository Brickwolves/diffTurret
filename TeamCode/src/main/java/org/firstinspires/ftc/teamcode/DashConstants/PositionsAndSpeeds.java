package org.firstinspires.ftc.teamcode.DashConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PositionsAndSpeeds {

    @Config
    public static class SlidePositions {
        public static int slidesHighJunction = 700;
        public static int slidesHighFront = 620;//9000
        public static int slidesHighFunny = 490;
        public static int slidesMidJunction = 380;  //6500
        public static int slidesMidFunny = 150;
        public static int slidesMidFront = 310;
        public static int slidesDepositDrop = 20;  //1000
        public static int slidesTippedHeight = 0;
        public static int slidesStackIncrease = 45;
    }
    @Config
    public static class V4BPositions {
        //works in degrees
        public static int v4bDown = -60;
        public static int v4bHide = -20;
        public static int v4bDownFunny = -30;
        public static int v4bScoreBack = 170;
        public static int v4bScoreBackLow = 0;
        public static int v4bScoreFront = 18;
        public static int v4bScoreFrontLow = 14;
        public static int v4bStartAuto = -60;
        public static int v4b0 = -226;
        public static int v4b90 = -520;
        public static double v4bSpeed = 0.6;
        public static int v4bUndershoot = 30;
    }

    @Config
    public static class grabberPositions {
        public static double grabberDown = 0.93 ;
        public static double grabberScore = 0.95;
        public static double grabberScoreFront = 0.91;
        public static double grabberScoreFunnyFront = 0.8;
        public static double grabberScoreFunny = 0.84;
        public static double grabberTip = 0.98;
        public static double grabberHide = 0.88;
        public static double grabberStartAuto = 0.86;
        public static double grabber0 = 0.5;
        public static double grabber90 = 0.6;
    }


    public static double rateOfChange = 100;


    public static double clawOpen = 0.36;
    public static double clawOpenScore = 0.28;
    public static double clawTipped = 0.3;
    public static double clawClose = 0;

    public static double tipAngle = 3;

    public static double driveSpeed = 0.7;

    public static int stackedHeight = 1;



}

