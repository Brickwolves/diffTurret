package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CIRCLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_DN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB2;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.rateOfChange;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesHighJunction;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bScoreBack;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.IMU_DATUM;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.proportionalWeight;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Utilities.PID;


@TeleOp(name="Test TeleOp", group="Iterative Opmode")
public class TestTeleOp extends OpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private PID pid;
    private double setPoint = 0;
    private boolean wasTurning;
    private boolean pid_on = false;
    private boolean pid_on_last_cycle = false;
    private boolean KETurns = false;
    public boolean isTipped = false;
    public boolean idk = false;


    // Declare OpMode members.
    Robot robot;
    Controller controller;
    Controller controller2;



    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        setOpMode(this);


        pid = new PID(proportionalWeight, integralWeight, derivativeWeight);


        robot = new Robot(true);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        /*
                    Y O U R   C O D E   H E R E
                                                    */


        robot.gyro.setDatum(IMU_DATUM);


        multTelemetry.addData("Status", "Initialized");
        multTelemetry.addData("imu datum", IMU_DATUM);
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        Controller.update();
        robot.scorer.intake.updateSpeed();

        double power;

        //PID and Kinetic Turning
        double rotation = controller.get(RIGHT, X);

        // Turn off PID if we manually turn
        // Turn on PID if we're not manually turning and the robot's stops rotating
        double currentRateOfChange = robot.gyro.rateOfChange();
        if (rotation != 0) {
            pid_on = false;
        } else if (currentRateOfChange <= rateOfChange) pid_on = true;

        if(controller2.get(DPAD_UP,TAP)){
            idk = true;
        }else if(controller2.get(DPAD_DN,TAP)){
            idk = false;
        }

        if(controller.get(CIRCLE,TOGGLE)){
            if(!idk) {
                robot.scorer.open(false);
            }else{
                robot.scorer.openScore();
            }
        }else{
            robot.scorer.close();
        }

        if(controller2.get(TRIANGLE,TOGGLE)){
            robot.scorer.intake.runIntake(0.8);
        }else{
            robot.scorer.intake.runIntake(0);
        }

        if(idk){
            robot.scorer.grabber(grabberScore);
            robot.scorer.slides(1,slidesHighJunction);
            robot.scorer.v4b(v4bScoreBack);
        }else{
            robot.scorer.grabber(grabberDown);
            robot.scorer.slides(1,0);
            robot.scorer.v4b(v4bDown);
        }

        // Lock the heading if we JUST turned PID on
        // Correct our heading if the PID has and is still on
        if (pid_on && !pid_on_last_cycle) setPoint = robot.gyro.getAngle();
        else if (pid_on) rotation = pid.update(robot.gyro.getAngle() - setPoint, true);

        // Update whether the PID was on or not
        pid_on_last_cycle = pid_on;


        //DRIVING
        controller.setJoystickShift(LEFT, robot.gyro.getAngle());


        double drive = controller.get(LEFT, INVERT_SHIFTED_Y);
        double strafe = controller.get(LEFT, SHIFTED_X);
        double turn = controller.get(RIGHT, X);


        if (controller.get(LB1, ButtonControls.ButtonState.DOWN) || controller.get(LB2, DOWN)) {
            power = 0.3;
        } else {
            power = 0.8;
        }


        robot.drivetrain.setDrivePower(drive, strafe, rotation, power);





        //SIDE
        Side.red = !controller2.get(RB1, TOGGLE);
    /*
         ----------- L O G G I N G -----------
                                            */
        multTelemetry.addData("intake speed", robot.scorer.intake.getSpeed());
        multTelemetry.addData("Slide 1",robot.scorer.spool.getCurrentPosition());
        multTelemetry.addData("Slide 1",robot.scorer.spool2.getCurrentPosition());
        multTelemetry.update();
    }


    @Override
    public void stop(){

        /*
                    Y O U R   C O D E   H E R E
                                                   */

    }
}
