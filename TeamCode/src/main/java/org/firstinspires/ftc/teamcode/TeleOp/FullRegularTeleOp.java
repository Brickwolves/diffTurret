package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TAP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.ButtonState.TOGGLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.CIRCLE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_DN;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_L;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.DPAD_UP;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.LB2;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.SQUARE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;

import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.driveSpeed;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.rateOfChange;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.tipAngle;
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
import org.firstinspires.ftc.teamcode.Utilities.Files.BlackBox.BlackBoxLogger;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Utilities.PID;

//@Disabled
@TeleOp(name="Regular TeleOp", group="Iterative Opmode")
public class FullRegularTeleOp extends OpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private PID pid;
    private double setPoint = 0;
    private boolean pid_on = false;
    private boolean pid_on_last_cycle = false;
    public boolean isTipped = false;
    public String coneTipped = "Straight";
    public boolean isFunny = false;

    public BlackBoxLogger logger;

    public enum ScoreState {
        DOWN,
        TIPPED_FORWARDS, TIPPED_BACKWARDS,
        SCORE_HIGH, SCORE_MID, SCORE_LOW,
        SCORE_FRONT_HIGH, SCORE_FRONT_MID, SCORE_FRONT_LOW,
    }

    public ScoreState score = ScoreState.DOWN;


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

        logger = new BlackBoxLogger();


        multTelemetry.addData("Status", "Initialized");
        multTelemetry.addData("imu datum", IMU_DATUM);
        multTelemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


        robot.scorer.deposit(coneTipped);
        multTelemetry.addData("Tipping?", isTipped);
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

        isTipped = robot.gyro.getTipAngle() > tipAngle || robot.gyro.getTipAngle() < -tipAngle;

        if(isTipped){
            score = ScoreState.DOWN;
            robot.scorer.time.reset();
        }

        if(score == ScoreState.DOWN) {
            if (controller.get(CIRCLE, TOGGLE)) {
                robot.scorer.open(isFunny);
            } else {
                robot.scorer.close(true);
            }
        }





        //Scoring

        if(controller.get(DPAD_UP,TAP)){
            coneTipped = "Straight";
            isFunny = false;
        }
        if(controller.get(DPAD_DN, TAP)){
            coneTipped = "Forwards";
            isFunny = true;

        }



        switch(score){
            case DOWN:
                if(!isTipped) {
                    robot.scorer.deposit(coneTipped);
                }else{
                    robot.scorer.crashSlides();
                }
                break;
            case SCORE_LOW:
                robot.scorer.lowBack(isFunny);
                break;
            case SCORE_MID:
                robot.scorer.midBack(isFunny);
                break;
            case SCORE_HIGH:
                robot.scorer.highBack(isFunny);
                break;
            case SCORE_FRONT_LOW:
                robot.scorer.lowFront(isFunny);
                break;
            case SCORE_FRONT_MID:
                robot.scorer.midFront(isFunny);
                break;
            case SCORE_FRONT_HIGH:
                robot.scorer.highFront(isFunny);
                break;
        }


        if (controller.get(SQUARE, TAP) && score != ScoreState.DOWN && !isTipped) {
            coneTipped = "Straight";
            isFunny = false;
            score = ScoreState.DOWN;
            robot.scorer.time.reset();
        }

        if (controller2.get(DPAD_DN, TAP) && score != ScoreState.SCORE_FRONT_LOW && !isTipped) {
            score = ScoreState.SCORE_FRONT_LOW;
            robot.scorer.time.reset();
        }

        if (controller2.get(DPAD_L, TAP) && score != ScoreState.SCORE_MID && !isTipped) {
            score = ScoreState.SCORE_MID;
            robot.scorer.time.reset();
        }

        if (controller2.get(DPAD_UP, TAP) && score != ScoreState.SCORE_HIGH && !isTipped) {
            score = ScoreState.SCORE_HIGH;
            robot.scorer.time.reset();
        }

        if(controller2.get(TRIANGLE, TAP)){
            if(score == ScoreState.SCORE_LOW) {
                score = ScoreState.SCORE_FRONT_LOW;
                robot.scorer.time.reset();
            }
            else if(score == ScoreState.SCORE_MID){
                score = ScoreState.SCORE_FRONT_MID;
                robot.scorer.time.reset();
            }
            else if(score == ScoreState.SCORE_HIGH){
                score = ScoreState.SCORE_FRONT_HIGH;
                robot.scorer.time.reset();
            }
            else if(score == ScoreState.SCORE_FRONT_LOW) {
                score = ScoreState.SCORE_LOW;
                robot.scorer.time.reset();
            }
            else if(score == ScoreState.SCORE_FRONT_MID){
                score = ScoreState.SCORE_MID;
                robot.scorer.time.reset();
            }
            else if(score == ScoreState.SCORE_FRONT_HIGH){
                score = ScoreState.SCORE_HIGH;
                robot.scorer.time.reset();
            }
        }



        //IMU RESET
//        if (controller.get(CROSS, TAP)) {
//            robot.gyro.reset();
//        }

        //TURN WRAPPING
        //PID and Kinetic Turning
        double rotation = controller.get(RIGHT, X);

        // Turn off PID if we manually turn
        // Turn on PID if we're not manually turning and the robot's stops rotating
        double currentRateOfChange = robot.gyro.rateOfChange();
        if (rotation != 0) {
            pid_on = false;
        } else if (currentRateOfChange <= rateOfChange) pid_on = true;


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


        if (controller.get(LB1, ButtonControls.ButtonState.DOWN) || controller.get(LB2, DOWN)) {
            power = 0.3;
        } else {
            power = driveSpeed;
        }

        if(score != ScoreState.DOWN){
            power = .3;
        }


        robot.drivetrain.setDrivePower(drive, strafe, rotation, power);





        //SIDE
        Side.red = !controller2.get(RB1, TOGGLE);
    /*
         ----------- L O G G I N G -----------
                                            */

        logger.log("RotInput", rotation, 3);
        logger.log("ThumbstickRot", controller.get(RIGHT, X), 3);
        logger.log("GyroReading", robot.gyro.getAngle(), 2);
        logger.writeData();

        multTelemetry.addData("Score State", score);
        multTelemetry.addData("Funny", isFunny);
        multTelemetry.addData("Slides Height", robot.scorer.spool.getCurrentPosition());
        multTelemetry.update();
    }


    @Override
    public void stop(){

        /*
                    Y O U R   C O D E   H E R E
                                                   */

    }
}
