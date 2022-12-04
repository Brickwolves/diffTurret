package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_DN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_L;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_R;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB2;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.rateOfChange;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.IMU_DATUM;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.proportionalWeight;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Loggers.Side;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;

@Disabled
@TeleOp(name="Servo TeleOp", group="Iterative Opmode")
public class BaseServoTeleOp extends OpMode {
    public Servo myServo;
    @Override
    public void init() {
        setOpMode(this);
        //ServoNameInConfig is whatever you set the name of the servo to in config
        myServo = hardwareMap.get(Servo.class, "ServoNameInConfig");
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        //Tune this value
        myServo.setPosition(1);
    }
    @Override
    public void loop() {
        //Tune this value too
        myServo.setPosition(0);
    }
    @Override
    public void stop(){

        /*
                    Y O U R   C O D E   H E R E
                                                   */

    }
}
