package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB2;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB2;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;

import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.Y;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.IMU_DATUM;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.proportionalWeight;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.rateOfChange;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Color_Sensor;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Utilities.PID;


@TeleOp(name="Rocket League", group="Iterative Opmode")
public class RocketLeague extends OpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime boostTimer = new ElapsedTime();
    public boolean isTipped = false;
    Color_Sensor colorSensor;
    public int charge;



    // Declare OpMode members.
    Robot robot;
    Controller controller;



    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        setOpMode(this);

        robot = new Robot(1);
        colorSensor = new Color_Sensor();
        colorSensor.init("color");


        controller = new Controller(gamepad1);
        /*
                    Y O U R   C O D E   H E R E
                                                    */





        multTelemetry.addData("Status", "Initialized");
        multTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        multTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();


        /*
                    Y O U R   C O D E   H E R E
                                                   */

        multTelemetry.addData("Status", "Started");
        multTelemetry.update();
        boostTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        Controller.update();
        colorSensor.updateRed();

        double power;



//        //PID and Kinetic Turning
//        double rotation = controller.get(RIGHT, X);
//
//        // Turn off PID if we manually turn
//        // Turn on PID if we're not manually turning and the robot's stops rotating
//        double currentRateOfChange = robot.gyro.rateOfChange();
//        if (rotation != 0) {
//            pid_on = false;
//        } else if (currentRateOfChange <= rateOfChange) pid_on = true;
//
//
//        // Lock the heading if we JUST turned PID on
//        // Correct our heading if the PID has and is still on
//        if (pid_on && !pid_on_last_cycle) setPoint = robot.gyro.getAngle();
//        else if (pid_on) rotation = pid.update(robot.gyro.getAngle() - setPoint, true);
//
//        // Update whether the PID was on or not
//        pid_on_last_cycle = pid_on;

        // BOOST
        if(colorSensor.getRedCacheValue()>1000){
            if(boostTimer.seconds()>3){
                charge++;
                boostTimer.reset();
            }
        }

        if(controller.get(RB2,DOWN)) {
            if (charge > 0) {
                if (runtime.seconds() % 1 == 0) {
                    charge--;
                }
                power = .9;
            } else {
                power = .4;
            }
        }else{
            power = .4;
        }



        //DRIVING



        double drive = controller.get(LEFT, INVERT_Y);
        double strafe = controller.get(LEFT, X);
        double turn = controller.get(RIGHT, X);


        robot.drivetrain.setDrivePower(drive,strafe,turn, power);





    /*
         ----------- L O G G I N G -----------
                                            */
        multTelemetry.addData("Tipping?", isTipped);
        multTelemetry.addData("Red", colorSensor.getRedCacheValue());
        multTelemetry.addData("Charge",charge);
        multTelemetry.update();
    }


    @Override
    public void stop(){

        /*
                    Y O U R   C O D E   H E R E
                                                   */

    }
}
