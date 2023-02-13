package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

public class AbsoluteEncoder {

    private AnalogInput encoder;

    public AbsoluteEncoder(String hardwareID){
        encoder = OpModeUtils.hardwareMap.get(AnalogInput.class, hardwareID);
    }

    public double getAngle() {
        return MathUtils.linearConversion(encoder.getVoltage() * 72, 1.3, 348, 0, 360);
    }
}
