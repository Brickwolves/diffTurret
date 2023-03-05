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
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.OPTIONS;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB1;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.RB2;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.SHARE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.SQUARE;
import static org.firstinspires.ftc.teamcode.Controls.ButtonControls.Input.TRIANGLE;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.LEFT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Input.RIGHT;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_SHIFTED_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;

import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.Y;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.SlidePositions.slidesAdjust;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.V4BAdjust;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bScoreFrontLow;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.driveSpeed;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberAdjust;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberDown;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberScore;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberScoreFront;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberScoreFunny;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.grabberPositions.grabberTip;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.rateOfChange;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.stackedHeight;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.tipAngle;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.V4BPositions.v4bScoreBack;
import static org.firstinspires.ftc.teamcode.Hardware.Scoring.beaconScore;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.IMU_DATUM;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.slidesOffset;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.v4bOffset;

import static org.firstinspires.ftc.teamcode.Utilities.Constants.grabberOffset;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.ControllerWeights.proportionalWeight;
import static org.firstinspires.ftc.teamcode.Utilities.NonConstants.fullyDown;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Control.Hardware.V6Hardware;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Control.Vector2d;
import org.firstinspires.ftc.teamcode.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Side;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.revextensions2.ExpansionHubEx;

import java.util.List;

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
    public boolean manualClaw = false;
    public boolean clawOpen = false;
    public boolean cacheBeam;
    public boolean completeManual = false;

    private boolean pressed = false;


    public enum ScoreState {
        DOWN,
        TIPPED_FORWARDS, TIPPED_BACKWARDS,
        SCORE_HIGH, SCORE_MID, SCORE_LOW,
        SCORE_FRONT_HIGH, SCORE_FRONT_MID, SCORE_FRONT_LOW,
        STACKED_HEIGHT, ESCAPE
    }

    public ScoreState score = ScoreState.DOWN;


    // Declare OpMode members.
    Robot robot;
    Controller controller;
    Controller controller2;


    List<LynxModule> hubs;



    /*
     * Code to run ONCE when the driver hits INIT
     */


    Movement drive;
    @Override
    public void init() {
        setOpMode(this);

        V6Hardware hardware = new V6Hardware(hardwareMap);

        drive = new Movement(0,0, hardware);

        stackedHeight = 1;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub: hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        pid = new PID(proportionalWeight, integralWeight, derivativeWeight);


        robot = new Robot(true);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        beaconScore = false;
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
        for (LynxModule hub: hubs) {
            hub.clearBulkCache();
        }

        robot.scorer.startTeleop(coneTipped);
        multTelemetry.addData("Tipping?", isTipped);
        multTelemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        //TODO never delete this it makes your code really good
        v4bOffset = 0;


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

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        Controller.update();
        robot.scorer.intake.updateSpeed();
        if(controller2.get(RB1,TAP) && controller2.get(LB1,TAP)){
            completeManual = !completeManual;
        }

            robot.scorer.resetAutomatic();

            double power;


            //ANTI TIP CODE

            isTipped = robot.gyro.getTipAngle() > tipAngle || robot.gyro.getTipAngle() < -tipAngle;

            if (isTipped) {
                score = ScoreState.DOWN;
                robot.scorer.time.reset();
            }

            manualClaw = true;//controller.get(TRIANGLE, TOGGLE);

            clawOpen = (/*robot.scorer.updateBeam() || */controller.get(CIRCLE, TAP)) != clawOpen;


            if (score == ScoreState.DOWN || score == ScoreState.STACKED_HEIGHT) {
                if (!manualClaw && !isFunny) {
                    if (fullyDown) {
                        if (clawOpen) {
                            robot.scorer.open(false);
                        } else {
                            robot.scorer.close();
                        }
                    }
                } else {
                    if (controller.get(CIRCLE, TOGGLE)) {
                        robot.scorer.open(isFunny);
                    } else {
                        robot.scorer.close();
                    }
                }
            }




            //Scoring

            if (controller.get(DPAD_UP, TAP)) {
                coneTipped = "Straight";
                robot.scorer.intake.runIntake(0);
                isFunny = false;
            }
            if (controller.get(DPAD_DN, TAP)) {
                coneTipped = "Forwards";
                if (score == ScoreState.DOWN) {
                    robot.scorer.intake.runIntake(0);
                } else {
                    robot.scorer.intake.runIntake(0);
                }
                isFunny = true;
            }


            if(!pressed && gamepad2.right_bumper){
                //increment
                pressed = true;
            }else if(!pressed && gamepad2.left_bumper){

                pressed = true;
            }else if(!gamepad2.left_bumper && !gamepad2.right_bumper){
                pressed = false;
            }


//IVAN CODE
    //Grabber Orientation
        //Increase
        if(controller2.get(RB2, DOWN)){
            v4bOffset += V4BAdjust;
            v4bDown += V4BAdjust;
        //Decrease
        } else if(controller2.get(LB2, DOWN)){
            v4bOffset -= V4BAdjust;
            v4bDown -= V4BAdjust;
        }


    //Slides
        //Increase
        if(controller2.get(LB1, DOWN)){
            slidesOffset += slidesAdjust;
            //Decrease
        } else if(controller2.get(RB1, DOWN)){
            slidesOffset -=slidesAdjust;
        }

    //V4B
        if(controller2.get(SHARE, TAP)){
            grabberOffset -= grabberAdjust;
        } else if(controller2.get(OPTIONS,TAP)){
            grabberOffset += grabberAdjust;
        }


            switch (score) {
                case DOWN:
                    if (!isTipped) {
                        robot.scorer.deposit(coneTipped);
                    } else {
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
                case STACKED_HEIGHT:
                    robot.scorer.stackPickup(stackedHeight);
                    break;
                case ESCAPE:
                    robot.scorer.stackEscape(stackedHeight);
                    break;
            }

            if (controller.get(RB1, TAP) && score == ScoreState.STACKED_HEIGHT && stackedHeight < 5) {
                stackedHeight += 1;
            }

            if (controller.get(LB1, TAP) && score == ScoreState.STACKED_HEIGHT && stackedHeight > 1) {
                stackedHeight -= 1;
            }

            if (controller.get(SQUARE, TAP) && score != ScoreState.DOWN && !isTipped) {
                beaconScore = false;
                coneTipped = "Straight";
                isFunny = false;
                if (score == ScoreState.ESCAPE) {
                    stackedHeight--;
                }
                score = ScoreState.DOWN;
                robot.scorer.time.reset();
            }

            if (controller.get(TRIANGLE, TAP) && score != ScoreState.DOWN && !isTipped) {
                beaconScore = true;
                robot.scorer.openScore();
            }


            if (controller.get(RB2, TAP)) {
                if (!isTipped && !isFunny) {
                    if (score != ScoreState.STACKED_HEIGHT) {
                        score = ScoreState.STACKED_HEIGHT;
                        robot.scorer.time.reset();
                        isFunny = false;
                        coneTipped = "Straight";
                    } else {
                        score = ScoreState.ESCAPE;
                        robot.scorer.time.reset();
                    }
                }
            }


            if (controller2.get(DPAD_DN, TAP) && (score == ScoreState.DOWN || score == ScoreState.ESCAPE) && !isTipped) {
                score = ScoreState.SCORE_LOW;
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

            if (controller2.get(CROSS, TAP) && (score == ScoreState.DOWN || score == ScoreState.ESCAPE) && !isTipped) {
                score = ScoreState.SCORE_FRONT_LOW;
                robot.scorer.time.reset();
            }

            if (controller2.get(SQUARE, TAP) && score != ScoreState.SCORE_FRONT_MID && !isTipped) {
                score = ScoreState.SCORE_FRONT_MID;
                robot.scorer.time.reset();
            }

            if (controller2.get(TRIANGLE, TAP) && score != ScoreState.SCORE_FRONT_HIGH && !isTipped) {
                score = ScoreState.SCORE_FRONT_HIGH;
                robot.scorer.time.reset();
            }


            if (score != ScoreState.DOWN) {
                clawOpen = true;
            }

            //IMU RESET
//        if (controller.get(CROSS, TAP)) {
//            robot.gyro.reset();
//        }

            //TURN WRAPPING
            //PID and Kinetic Turning
            double rotation = controller.get(RIGHT, X);


            //Gyro Reset
            if (controller.get(CROSS, TAP)) {
                robot.gyro.reset();
                pid_on = false;
            }
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


            if (controller.get(LB2, DOWN)) {
                power = 0.3;
            } else {
                power = driveSpeed;
            }

            if (score != ScoreState.DOWN) {
                if (score == ScoreState.STACKED_HEIGHT) {
                    power = 0.6;
                } else {
                    power = .45;
                }
            }


            robot.drivetrain.setDrivePower(drive, strafe, rotation, power);


            //SIDE
            Side.red = !controller2.get(RB1, TOGGLE);

            //RUMBLE
            if (robot.scorer.updateBeam() != cacheBeam) {
                controller.rumble(1);
                controller2.rumble(1);
            }
            cacheBeam = robot.scorer.updateBeam();

    /*
         ----------- L O G G I N G -----------
                                            */

//        logger.log("RotInput", rotation, 3);
//        logger.log("ThumbstickRot", controller.get(RIGHT, X), 3);
//        logger.log("GyroReading", robot.gyro.getAngle(), 2);
//        logger.log("RawIMU", robot.gyro.rawAngle(), 3);
//        logger.writeData();

            multTelemetry.addData("Score State", score);
            multTelemetry.addData("slidesOffset", slidesOffset);
            multTelemetry.addData("v4bOffset", v4bOffset);
            multTelemetry.addData("grabberOffset", grabberOffset);
            //multTelemetry.addData("Servo Bus MilliAmps",controlHub.getServoBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS));


            multTelemetry.addData("Funny", isFunny);
            multTelemetry.addData("Slides Height", -robot.scorer.spool.getCurrentPosition());
            multTelemetry.addData("is beam broken", robot.scorer.beamBroken());
            multTelemetry.addData("Intake Speed", robot.scorer.intake.getSpeed());

            multTelemetry.update();





    }


    @Override
    public void stop(){

        /*
                    Y O U R   C O D E   H E R E
                                                   */

    }
}
