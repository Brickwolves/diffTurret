package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.Files.Clocks.Clock;
import org.firstinspires.ftc.teamcode.Utilities.Files.Clocks.SystemClockMillis;
import org.firstinspires.ftc.teamcode.Utilities.Files.FileUtils.FileLogWriter;
import org.firstinspires.ftc.teamcode.Utilities.Files.Loggers.GridLogger;

@Disabled
@TeleOp(name="ArmLogOpmode", group="Iterative Opmode")
public class ArmLoggingOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm;

    private GridLogger logger;
    private FileLogWriter logWriter;
    private Clock systemClock = new SystemClockMillis();
    @Override
    public void init() {
        setOpMode(this);
        //ServoNameInConfig is whatever you set the name of the servo to in config
        arm = hardwareMap.get(DcMotor.class, "v4b");
        logWriter = new FileLogWriter("armLog");
        logger = new GridLogger(logWriter, systemClock);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }
    @Override
    public void loop() {

        arm.setPower(gamepad1.right_stick_x);
        logger.add("pow", arm.getPower());
        logger.add("veloc", ((DcMotorEx) arm).getVelocity());
        logger.writeRow();
        multTelemetry.addData("pow", arm.getPower());
        multTelemetry.addData("veloc", ((DcMotorEx) arm).getVelocity());
        multTelemetry.update();


    }
    @Override
    public void stop(){
        logger.stop();

        /*
                    Y O U R   C O D E   H E R E
                                                   */

    }
}
