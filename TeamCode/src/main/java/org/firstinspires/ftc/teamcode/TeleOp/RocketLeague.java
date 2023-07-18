package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB2;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Color_Sensor;



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

        // BOOST
        if(colorSensor.getRedCacheValue()>1000){
            if(boostTimer.seconds()>3){
                charge++;
                boostTimer.reset();
            }
        }

        if(controller.get(RB2,DOWN)) {
            if (charge > 0) {
                if (runtime.seconds()>1) {
                    charge--;
                    runtime.reset();
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
