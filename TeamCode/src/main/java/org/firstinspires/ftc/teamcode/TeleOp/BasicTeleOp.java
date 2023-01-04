package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CIRCLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CROSS;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_DN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_L;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_R;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB2;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB2;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.SQUARE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.rateOfChange;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.tipAngle;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.v4bScoreBack;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.IMU_DATUM;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.proportionalWeight;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Loggers.Side;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;

@Disabled
@TeleOp(name="Basic TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends OpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private PID pid;
    private double setPoint = 0;
    private boolean wasTurning;
    private boolean pid_on = false;
    private boolean pid_on_last_cycle = false;
    private boolean KETurns = false;
    public boolean isTipped = false;

    public enum SlidesState {HIGH, MIDDLE, LOW, GROUND, DOWN, DEPOSIT}
    public enum V4B {DOWN,TIPPED,SCORE,SCOREFUNNY}

    public EnhancedTeleOp.SlidesState slidesState = EnhancedTeleOp.SlidesState.DOWN;

    public V4B v4b = V4B.DOWN;


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


        robot = new Robot();
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


        robot.grabber.rotateGrabber(0);
        robot.grabber.rotate(v4bDown);
        v4b = V4B.DOWN;

        robot.grabber.open();

        multTelemetry.addData("Status", "InitLoop");
        multTelemetry.addData("imu datum", IMU_DATUM);
        multTelemetry.addData("Tipping?", isTipped);
        multTelemetry.addData("Grabber State", v4b);
        multTelemetry.addData("Grabber Pos", robot.grabber.grabberSpin.getPosition());
        multTelemetry.addData("v4b1", robot.grabber.v4b1.getPosition());
        multTelemetry.addData("v4b2", robot.grabber.v4b2.getPosition());
        multTelemetry.addData("claw", robot.grabber.squeezer.getPosition());
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

        double power;




        //ANTI TIP CODE

        if(robot.gyro.getTipAngle() > tipAngle || robot.gyro.getTipAngle() < -tipAngle){
            isTipped = true;
        }else{
            isTipped = false;
        }

        if(isTipped){
            robot.grabber.down();
        }

        // if right trigger send slides down open intake
        if (controller.get(CIRCLE, TOGGLE)) {
            robot.grabber.open();
            //if right button close intake
        } else {
            robot.grabber.close();
        }


        //PID and Kinetic Turning
        double rotation = controller.get(RIGHT, X);

        // Turn off PID if we manually turn
        // Turn on PID if we're not manually turning and the robot's stops rotating
        double currentRateOfChange = robot.gyro.rateOfChange();
        if (rotation != 0) {
            pid_on = false;
        } else if (currentRateOfChange <= rateOfChange) pid_on = true;



        //Scoring

        if(controller2.get(TRIANGLE, TAP)){
            switch(v4b){
                //TIPPED
//                case DOWN:
//                    robot.grabber.rotateGrabber(grabberTip);
//                    robot.grabber.rotate(v4bTipped);
//                    v4b = TIPPED;
//                    break;
                //SCORE
                case DOWN:
                    robot.grabber.rotateGrabber(0);
                    robot.grabber.rotate(v4bScoreBack);
                    v4b = V4B.SCORE;
                    break;
                //SCORE FUNNY
//                case SCORE:
//                    robot.grabber.rotateGrabber(grabberScoreFunny);
//                    robot.grabber.rotate(v4bScore);
//                    v4b = V4B.SCOREFUNNY;
//                    break;
                //DOWN
                case SCORE:
                    robot.grabber.rotateGrabber(0);
                    robot.grabber.rotate(v4bDown);
                    v4b = V4B.DOWN;
                    break;

            }
        }



        if (controller2.get(DPAD_UP, TAP) && slidesState != EnhancedTeleOp.SlidesState.HIGH && !isTipped) {
            slidesState = EnhancedTeleOp.SlidesState.HIGH;
            robot.grabber.time.reset();
        }

        if (controller2.get(DPAD_L, TAP) && slidesState != EnhancedTeleOp.SlidesState.MIDDLE && !isTipped) {
            slidesState = EnhancedTeleOp.SlidesState.MIDDLE;
            robot.grabber.time.reset();
        }

        if (controller2.get(DPAD_R, TAP) && slidesState != EnhancedTeleOp.SlidesState.LOW && !isTipped) {
            slidesState = EnhancedTeleOp.SlidesState.LOW;
            robot.grabber.time.reset();
        }

        if (controller2.get(DPAD_DN, TAP) && slidesState != EnhancedTeleOp.SlidesState.GROUND && !isTipped) {
            slidesState = EnhancedTeleOp.SlidesState.GROUND;
            robot.grabber.time.reset();
        }

        if (controller2.get(CIRCLE, TAP) && slidesState != EnhancedTeleOp.SlidesState.DEPOSIT && !isTipped) {
            robot.grabber.wentDown = false;
            slidesState = EnhancedTeleOp.SlidesState.DEPOSIT;
            robot.grabber.time.reset();
        }

        if (controller2.get(SQUARE, TAP) && slidesState != EnhancedTeleOp.SlidesState.DOWN && !isTipped) {
            slidesState = EnhancedTeleOp.SlidesState.DOWN;
            robot.grabber.time.reset();
        }

        // if right trigger send slides down open intake
        if (controller.get(RB1, TAP)) {
            slidesState = EnhancedTeleOp.SlidesState.DOWN;
            robot.grabber.open();
            //if right button close intake
        } else if (controller.get(RB2, TAP)) {
            robot.grabber.close();
        }

        switch (slidesState) {
            case HIGH:
                robot.grabber.high();
                break;
            case MIDDLE:
                robot.grabber.middle();
                break;
            case LOW:
                robot.grabber.low();
                break;
            case GROUND:
                robot.grabber.ground();
                break;
            case DEPOSIT:
                robot.grabber.deposit();
                break;
            case DOWN:
                robot.grabber.down();
        }

        //IMU RESET
        if (controller.get(CROSS, TAP)) {
            robot.gyro.reset();
        }

        //TURN WRAPPING
        if (controller.get(DPAD_R, TAP)) {
            setPoint = MathUtils.closestAngle(270, robot.gyro.getAngle());
            pid_on = true;
            KETurns = false;
        } else if (controller.get(DPAD_L, TAP)) {
            setPoint = MathUtils.closestAngle(90, robot.gyro.getAngle());
            pid_on = true;
            KETurns = false;
        } else if (controller.get(DPAD_UP, TAP)) {
            setPoint = MathUtils.closestAngle(0, robot.gyro.getAngle());
            pid_on = true;
            KETurns = false;
        } else if (controller.get(DPAD_DN, TAP)) {
            setPoint = MathUtils.closestAngle(180, robot.gyro.getAngle());
            pid_on = true;
            KETurns = false;

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
        multTelemetry.addData("Tipping?", isTipped);
        multTelemetry.addData("slide target", robot.grabber.spool.getTargetPosition());
        multTelemetry.addData("slide current", robot.grabber.spool.getCurrentPosition());
        multTelemetry.update();
    }


    @Override
    public void stop(){

        /*
                    Y O U R   C O D E   H E R E
                                                   */

    }
}
