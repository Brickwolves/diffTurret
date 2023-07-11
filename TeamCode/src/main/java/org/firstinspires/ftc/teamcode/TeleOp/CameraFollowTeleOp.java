package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB2;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.rateOfChange;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.yd;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.yi;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.yp;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.IMU_DATUM;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.proportionalWeight;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Camera;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.Side;

@TeleOp(name="Camera TeleOp", group="Iterative Opmode")
public class CameraFollowTeleOp extends OpMode {

    Camera cam;
    DcMotor spin;
    public double x;
    public double y;
    public double size;
    //TODO MAYBE USE PID TO TARGET
    public PID yellowPID;
    @Override
    public void init() {
        setOpMode(this);
        cam = new Camera("webcam");
        spin = hardwareMap.get(DcMotor.class,"spin");
        yellowPID = new PID(yp,yi,yd);

    }

    @Override
    public void init_loop() {
        multTelemetry.addData("x", cam.pipeline.getTargetX());
        multTelemetry.addData("y", cam.pipeline.getTargetY());
        multTelemetry.addData("size",  cam.pipeline.checkSize());
        multTelemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        x = cam.pipeline.getTargetX();
        y = cam.pipeline.getTargetY();
        size = cam.pipeline.checkSize();

        yellowPID.setWeights(yp,yi,yd);



        double pidVal = MathUtils.interpolateRanges(x,0,280,-50,50);
        double power = yellowPID.update(pidVal,false);

        multTelemetry.addData("x", x);
        multTelemetry.addData("y", y);
        multTelemetry.addData("size", size);
        multTelemetry.addData("","");
        multTelemetry.addData("interpolated x",pidVal);
        multTelemetry.addData("power",power);
        multTelemetry.update();

        if(size>100) {


            spin.setPower(-power);
        }
    }

    @Override
    public void stop(){

    }
}
