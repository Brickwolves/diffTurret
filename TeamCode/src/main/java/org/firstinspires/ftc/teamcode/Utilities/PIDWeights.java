package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDWeights {

    public static double proportionalWeight = 0.012;
    public static double integralWeight = 0;
    public static double derivativeWeight = 0.000011;

    public static double tP = 5.5;
    public static double tI = 0;
    public static double tD = 0.002;
}