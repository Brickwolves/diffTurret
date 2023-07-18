package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.angleMode.RADIANS;
import static java.lang.Math.atan;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static java.lang.StrictMath.floorMod;

import android.os.Build;

import org.opencv.core.Point;

import androidx.annotation.RequiresApi;




public class MathUtils {

    /**
     * @param targetAngle
     * @param currentAngle
     * @return `````````````````the closest relative target angle
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double closestAngle(double targetAngle, double currentAngle) {
        double simpleTargetDelta = floorMod(Math.round((360 - targetAngle) + currentAngle), 360);
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
    }

    public static double closestAngle(double targetAngle, double currentAngle, boolean ticks) {
        double simpleTargetDelta = floorModDouble(Math.round((384.5 - targetAngle) + currentAngle), 384);
        double alternateTargetDelta = -1 * (384.5 - simpleTargetDelta);
        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
    }

    public static double floorModDouble(double dividend, int divisor){
        return floorMod(Math.round(dividend * 1e6), divisor) / 1e6;
    }

    public static Point shift(Point p, double shiftAngle){
        double rawX = p.x;
        double rawY = p.y;
        double x = (rawX * Math.cos(Math.toRadians(shiftAngle))) - (rawY * Math.sin(Math.toRadians(shiftAngle)));
        double y = (rawX * Math.sin(Math.toRadians(shiftAngle))) + (rawY * Math.cos(Math.toRadians(shiftAngle)));
        return new Point(x,y);
    }

    public static boolean inRange(double low, double input, double high){
        return input > low && input < high;
    }

    public static boolean inRange(int low, int input, int high){
        return input > low && input < high;
    }

    public static double linearConversion(double value, double oldMin, double oldMax, double newMin, double newMax){
        double oldRange = (oldMax - oldMin);
        double newRange = (newMax - newMin);
        return (((value - oldMin) * newRange) / oldRange) + newMin;
    }

    /**
     * @param x
     * @param a_min
     * @param a_max
     * @param b_min
     * @param b_max
     * @return x but mapped from [a_min, a_max] to [b_min, b_max]
     */
    public static double map(double x, double a_min, double a_max, double b_min, double b_max){
        return (x - a_min) / (a_max - a_min) * (b_max - b_min) + b_min;
    }

    /*
                        T R I G    E Q U A T I O N S

                                                                        */

    public enum angleMode {
        RADIANS, DEGREES
    }

    public static double cos(double value, angleMode mode){
        return (mode == RADIANS) ? Math.cos(value) : Math.cos(toRadians(value));
    }
    public static double sin(double value, angleMode mode){
        return (mode == RADIANS) ? Math.sin(value) : Math.sin(toRadians(value));
    }
    public static double tan(double value, angleMode mode){
        return (mode == RADIANS) ? Math.tan(value) : Math.tan(toRadians(value));
    }

    public static double acos(double value, angleMode mode){
        return (mode == RADIANS) ? Math.cos(value) : Math.cos(toRadians(value));
    }
    public static double asin(double value, angleMode mode){
        return (mode == RADIANS) ? Math.asin(value) : Math.sin(toRadians(value));
    }
    public static double atan(double value, angleMode mode){
        return (mode == RADIANS) ? Math.atan(value) : Math.tan(toRadians(value));
    }
    


    /*
                    C O N V E R S I O N S
                                                    */
    public static double lessThan1000TicksToCentimeters(double ticks){
        return (0.0748 * Math.pow(ticks, 2)) + (.677 * ticks) + 87.3;
    }


    public static double centimeters2Ticks(double c){
        return (0.118 * Math.pow(c, 2)) + (3.66 * c) + 7;
    }

    public static double ticks2Centimeters(double ticks){
        double rootTerm = 4 * 0.118 * (7 - ticks);
        if (rootTerm < 0) return 0;

        double numer = sqrt(Math.pow(3.66, 2) - rootTerm);
        double denom = 2 * 0.118;
        double answ1 = (-3.66 + numer) / denom;
        double answ2 = (-3.66 - numer) / denom;
        return (answ1 > 0) ? answ1 : answ2;
    }

    public static double convertInches2Ticks(double ticks){
        return (ticks - 4.38) / 0.0207; // Calculated using desmos
    }

    public static double convertTicks2Inches(double inches){
        return (0.0207 * inches) + 4.38; // Calculated using desmos
    }

    public static Point odoToTiles(Point odoInput){
        Point odoReturn = new Point();
        odoReturn.x = odoInput.x / 24;
        odoReturn.y = odoInput.y / 24;
        return odoReturn;
    }

    public static Point tilesToOdo(Point tilesInput){
        return tilesInput;
    }

    /*
    number n that is on range start1->end1 to range start2->end2
     */
    public static double interpolateRanges(double n, double start1, double end1, double start2, double end2){
        double ratio = (n-start1) / (end1-start1);
        return start2 + (ratio * (end2 - start2));
    }
}
