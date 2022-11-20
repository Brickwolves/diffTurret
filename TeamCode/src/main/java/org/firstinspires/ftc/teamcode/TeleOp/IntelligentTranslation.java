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
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.INVERT_Y;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.SHIFTED_X;
import static org.firstinspires.ftc.teamcode.Controls.JoystickControls.Value.X;
import static org.firstinspires.ftc.teamcode.DashConstants.PositionsAndSpeeds.rateOfChange;
import static org.firstinspires.ftc.teamcode.Utilities.Constants.IMU_DATUM;
import static org.firstinspires.ftc.teamcode.Utilities.ITUtils.ITD;
import static org.firstinspires.ftc.teamcode.Utilities.ITUtils.ITI;
import static org.firstinspires.ftc.teamcode.Utilities.ITUtils.ITP;
import static org.firstinspires.ftc.teamcode.Utilities.ITUtils.cachedPose;
import static org.firstinspires.ftc.teamcode.Utilities.ITUtils.junctions;
import static org.firstinspires.ftc.teamcode.Utilities.ITUtils.kDrive;
import static org.firstinspires.ftc.teamcode.Utilities.ITUtils.kHDist;
import static org.firstinspires.ftc.teamcode.Utilities.ITUtils.kStrafe;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.derivativeWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.integralWeight;
import static org.firstinspires.ftc.teamcode.Utilities.PIDWeights.proportionalWeight;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Loggers.Side;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.opencv.core.Point;

//@Disabled
    @TeleOp(name="Intelligent Translation", group="Iterative Opmode")
    public class IntelligentTranslation extends OpMode {

        // Declare OpMode members.
        private final ElapsedTime runtime = new ElapsedTime();
        private PID pid;
        private double setPoint = 0;
        private boolean pid_on = false;
        private boolean pid_on_last_cycle = false;
        private boolean KETurns = false;

        private PID ITPid;
        private double strafeChange;
        private double driveChange;
        private double ITPower;

        private double shiftedDrive;
        private double shiftedStrafe;








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



            robot.drivetrain.setPoseEstimate(new Pose2d(-30,70,Math.toRadians(270)));


            pid = new PID(proportionalWeight, integralWeight, derivativeWeight);
            ITPid = new PID(ITP,ITI,ITD);


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


            multTelemetry.addData("Status", "InitLoop");
            multTelemetry.addData("imu datum", IMU_DATUM);
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

            //PID and Kinetic Turning
            double rotation = controller.get(RIGHT, X);

            // Turn off PID if we manually turn
            // Turn on PID if we're not manually turning and the robot's stops rotating
            double currentRateOfChange = robot.gyro.rateOfChange();
            if (rotation != 0){ pid_on = false;}
            else if (currentRateOfChange <= rateOfChange) pid_on = true;


            //IMU RESET
            if(controller.get(CROSS, TAP)){
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



            //Intelligent translation
            cachedPose = robot.drivetrain.getPoseEstimate();

            for (int i = 0; i < 25; i++) {
                double yDist = Math.abs(junctions[i].getY() - cachedPose.getY());
                double xDist = Math.abs(junctions[i].getX() - cachedPose.getX());
                double hDist = Math.sqrt((yDist * yDist) + (xDist * xDist));
                strafeChange = kStrafe / xDist / (hDist*kHDist);
                driveChange = kDrive / yDist / (hDist*kHDist);

                //I still have to find a way to make it so that the values are negative when the junction is to the left or front of you
            }



            //DRIVING
            controller.setJoystickShift(LEFT, robot.gyro.getAngle());

            double drive = controller.get(LEFT, INVERT_Y) + driveChange;
            double strafe = controller.get(LEFT, X) + strafeChange;
            double turn = controller.get(RIGHT, X);

            Point changedAll = new Point(strafe,drive);


            shiftedDrive = MathUtils.shift(changedAll, robot.gyro.getAngle()).y;
            shiftedStrafe = MathUtils.shift(changedAll, robot.gyro.getAngle()).x;

            if(controller.get(LB1, ButtonControls.ButtonState.DOWN) || controller.get(LB2, DOWN)){
                power = 0.3;}
            else{
                power = 0.8;
            }




            robot.drivetrain.setDrivePower(shiftedDrive, shiftedStrafe, rotation, power);


            //SIDE
            Side.red = !controller2.get(RB1, TOGGLE);
    /*
         ----------- L O G G I N G -----------
                                            */
            multTelemetry.update();
        }


        @Override
        public void stop(){

        /*
                    Y O U R   C O D E   H E R E
                                                   */

        }
    }

