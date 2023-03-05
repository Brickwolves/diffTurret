package org.firstinspires.ftc.teamcode.DashConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PositionsAndSpeeds {

    @Config
    public static class SlidePositions {
        public static int slidesHighJunction = 685;
        public static int slidesHighFront = 570;//9000
        public static int slidesHighFunny = 490;
        public static int slidesMidJunction = 350;
        public static int slidesMidAuto = 315;
        public static int slidesMidFunny = 110;
        public static int slidesMidFront = 240;
        public static int slidesDepositDrop = 20;  //1000
        public static int slidesTippedHeight = 90;
        public static int slidesStackIncrease = 45;
        public static int slidesAdjust = 3;

        public static double SlidesP = .003, SlidesI = 0, SlidesD = .0002, F = .3;
    }
    @Config
    public static class V4BPositions {
        //works in degrees
        public static int v4bDown = -64 ;
        public static int v4bHide = -20;
        public static int v4bDownFunny = -40;
        public static int v4bScoreBack = 167;
        public static int v4bScoreBackAuto = 170;
        public static int v4bScoreBackLow = 185;
        public static int v4bScoreFront = 35;
        public static int v4bScoreFrontLow = 15;
        public static int v4bStartAuto = -60;
        public static int v4b0 = -226;
        public static int v4b90 = -520;
        public static double v4bSpeed = 0.6;
        public static int v4bUndershoot = 30;
        public static int V4BAdjust = 3;
    }

    @Config
    public static class grabberPositions {
        public static double grabberDown = 0.88;
        public static double grabberScore = 0.9;
        public static double grabberScoreAuto = 0.90;
        public static double grabberScoreFront = 0.86;
        public static double grabberScoreFunnyFront = 0.75;
        public static double grabberScoreFunny = 0.8;
        public static double grabberTip = 0.93;
        public static double grabberHide = 0.85;
        public static double grabberStartAuto = 0.81;
        public static double grabber0 = 0.5;
        public static double grabber90 = 0.6;
        public static double grabberAdjust = 0.01;
    }


    public static double rateOfChange = 100;


    public static double clawOpen = 0.36;
    public static double clawOpenScore = 0.28;
    public static double clawOpenBeacon = 0.2;
    public static double clawTipped = 0.3;
    public static double clawClose = 0;

    public static double braceOut = 1;
    public static double braceIn = 0.56;

    public static double tipAngle = 5;

    public static double driveSpeed = 0.7;

    public static int stackedHeight = 1;



}

