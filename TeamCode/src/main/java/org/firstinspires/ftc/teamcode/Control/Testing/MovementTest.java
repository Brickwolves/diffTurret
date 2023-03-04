package org.firstinspires.ftc.teamcode.Control.Testing;



import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Control.Hardware.V6Hardware;
import org.firstinspires.ftc.teamcode.Control.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.ClawIntake;

@Autonomous(name="Movement Test", group="Testing")
public class MovementTest extends LinearOpMode {

    public void runOpMode() {

        double y = 38;
        double x = 60;

//        double[][] path1points = {{0, 0}, {-38, 12}};
//        double[][] path2points = {{-38, 12}, {-130, 4.5}};
//        double[][] path12points = {{0, 0}, {-20, 15}, {-130, 4.5}};
//        double[][] path3points = {{-135, 4.5}, {-135, -53}};
//        double[][] path4points = {{-135, -53}, {-140, -25}, {-135, 4.5}};
//        double[][] path5points = {{0, 0}, {0, -200}};
//        double[][] path6points = {{0, -200}, {0, 0}};

        double[][] path1points = {{0, 0}, {0, y}, {-x, y}, {-x, 0}};
        double[][] path2points = {{-x, 0}, {-x, -y}, {-2*x, -y}, {-2*x, 0}};
        double[][] path3points = {{-2*x, 0}, {-2*x, y}, {-3*x, y}, {-3*x, 0}};
        double[][] path4points = {{-3*x, 0}, {-3*x, -y}, {-2*x, -y}, {-2*x, 0}};
        double[][] path5points = {{-2*x, 0}, {-2*x, y}, {-x, y}, {-x, 0}};
        double[][] path6points = {{-x, 0}, {-x, -y}, {0, -y}, {0, 0}};
        setOpMode(this);
        V6Hardware hardware = new V6Hardware(hardwareMap);
        ClawIntake intake = new ClawIntake(hardware);
        intake.slidesRetract();
        Movement drive = new Movement(0, 0, hardware);


        Movement.Function[] path1 = drive.getBezierCurve(path1points);
        Movement.Function[] path2 = drive.getBezierCurve(path2points);
        Movement.Function[] path3 = drive.getBezierCurve(path3points);
        Movement.Function[] path4 = drive.getBezierCurve(path4points);
//        Movement.Function[] path12 = drive.getBezierCurve(path12points);
        Movement.Function[] path5 = drive.getBezierCurve(path5points);
        Movement.Function[] path6 = drive.getBezierCurve(path6points);


        //initialize();

        initialize();
        waitForStart();

//        while (opModeIsActive()){
//            Hardware.rightFront.setPower(.05);
//            Hardware.rightBack.setPower(.05);
//            Hardware.leftFront.setPower(.05);
//            Hardware.leftBack.setPower(.05);
//
//            telemetry.addData("rightFront", Hardware.rightFront.getCurrentPosition());
//            telemetry.addData("leftFront", Hardware.leftFront.getCurrentPosition());
//            telemetry.addData("rightBack", Hardware.rightBack.getCurrentPosition());
//            telemetry.addData("leftBack", Hardware.leftBack.getCurrentPosition());
//            telemetry.update();
//        }

        boolean pressed1 = false;
        boolean pressed2 = false;

//        while (opModeIsActive()){
//            drive.odo.localize();
//           double[] location = drive.odo.getLocation();
//            if(gamepad1.dpad_up && !pressed1){
//                drive.odo.setPosition(location[0], location[1]-50);
//                pressed1 = true;
//            }else if(!gamepad1.dpad_up){
//                pressed1 = false;
//
//            }
//            if(gamepad1.dpad_down && !pressed2){
//                drive.odo.setPosition(location[0], location[1]+50);
//                pressed2 = true;
//            }else if(!gamepad1.dpad_down){
//                pressed2 = false;
//            }
////            drive.movementPID.setConstants(Movement.MovementDash.holdMoveP, Movement.MovementDash.holdMoveI, Movement.MovementDash.holdMoveD);
//            drive.holdMovementPID.update(-location[1]);
//            drive.headingPID.update(drive.catchJump(0, -drive.odo.getLocation()[2]));
//            drive.drive(0, 1, location[2], drive.holdMovementPID.getCorrection(), drive.headingPID.getCorrection());
//        }


//        while (opModeIsActive()){
//            drive.odo.localize();
//            drive.movementPID.update((Math.sqrt(Math.pow(drive.odo.getLocation()[0], 2) + Math.pow(drive.odo.getLocation()[1], 2))));
//            drive.headingPID.update(drive.catchJump(0, -drive.odo.getLocation()[2]));
//            drive.drive(0,1,drive.odo.getLocation()[2], drive.movementPID.getCorrection(), drive.headingPID.getCorrection());
//        }




            while (opModeIsActive() && drive.followPath(path1, Movement.MovementDash.testingPower, 0, .95,.002, 2000, .25, false, true)) {
                //System.out.println(", " + drive.odo.getLocation()[0] + ", " + drive.odo.getLocation()[1]);
                multTelemetry.addData("X", drive.odo.getLocation()[0]);
                multTelemetry.addData("Y", drive.odo.getLocation()[1]);
                telemetry.addData("Heading", drive.odo.getLocation()[2]);
                telemetry.addData("\t", " ");
                telemetry.addData("Veritical Encoder", drive.odo.getRawValues()[0]);
                telemetry.addData("Horizontal Encoder", drive.odo.getRawValues()[1]);
                telemetry.update();
                multTelemetry.update();

            }

            while (opModeIsActive() && drive.followPath(path2, Movement.MovementDash.testingPower, 0, .95,.002, 2000, .25, false, true)) {
                //System.out.println(", " + drive.odo.getLocation()[0] + ", " + drive.odo.getLocation()[1]);
                multTelemetry.addData("X", drive.odo.getLocation()[0]);
                multTelemetry.addData("Y", drive.odo.getLocation()[1]);
                telemetry.addData("Heading", drive.odo.getLocation()[2]);
                telemetry.addData("\t", " ");
                telemetry.addData("Veritical Encoder", drive.odo.getRawValues()[0]);
                telemetry.addData("Horizontal Encoder", drive.odo.getRawValues()[1]);
                telemetry.update();
                multTelemetry.update();
            }

        while (opModeIsActive() && drive.followPath(path3, Movement.MovementDash.testingPower, 0, .82,.002, 2000, .25, false,true)) {
            //System.out.println(", " + drive.odo.getLocation()[0] + ", " + drive.odo.getLocation()[1]);
            multTelemetry.addData("X", drive.odo.getLocation()[0]);
            multTelemetry.addData("Y", drive.odo.getLocation()[1]);
            telemetry.addData("Heading", drive.odo.getLocation()[2]);
            telemetry.addData("\t", " ");
            telemetry.addData("Veritical Encoder", drive.odo.getRawValues()[0]);
            telemetry.addData("Horizontal Encoder", drive.odo.getRawValues()[1]);
            telemetry.update();
            multTelemetry.update();
        }


        while (opModeIsActive() && drive.followPath(path4, .75, 0, .95,.002, 2000, .25, false,true)) {
            //System.out.println(", " + drive.odo.getLocation()[0] + ", " + drive.odo.getLocation()[1]);
            multTelemetry.addData("X", drive.odo.getLocation()[0]);
            multTelemetry.addData("Y", drive.odo.getLocation()[1]);
            telemetry.addData("Heading", drive.odo.getLocation()[2]);
            telemetry.addData("\t", " ");
            telemetry.addData("Veritical Encoder", drive.odo.getRawValues()[0]);
            telemetry.addData("Horizontal Encoder", drive.odo.getRawValues()[1]);
            telemetry.update();
            multTelemetry.update();
        }

        while (opModeIsActive() && drive.followPath(path5, .75, 0, .95,.002, 2000, .25, false,true)) {
            //System.out.println(", " + drive.odo.getLocation()[0] + ", " + drive.odo.getLocation()[1]);
            multTelemetry.addData("X", drive.odo.getLocation()[0]);
            multTelemetry.addData("Y", drive.odo.getLocation()[1]);
            telemetry.addData("Heading", drive.odo.getLocation()[2]);
            telemetry.addData("\t", " ");
            telemetry.addData("Veritical Encoder", drive.odo.getRawValues()[0]);
            telemetry.addData("Horizontal Encoder", drive.odo.getRawValues()[1]);
            telemetry.update();
            multTelemetry.update();
        }

        while (opModeIsActive() && drive.followPath(path6, .75, 0, .95,.002, .5, .25, true,true)) {
            //System.out.println(", " + drive.odo.getLocation()[0] + ", " + drive.odo.getLocation()[1]);
            multTelemetry.addData("X", drive.odo.getLocation()[0]);
            multTelemetry.addData("Y", drive.odo.getLocation()[1]);
            telemetry.addData("Heading", drive.odo.getLocation()[2]);
            telemetry.addData("\t", " ");
            telemetry.addData("Veritical Encoder", drive.odo.getRawValues()[0]);
            telemetry.addData("Horizontal Encoder", drive.odo.getRawValues()[1]);
            telemetry.update();
            multTelemetry.update();
        }

        while (opModeIsActive()) {
            drive.holdPosition(0,0,0);
            //System.out.println(", " + drive.odo.getLocation()[0] + ", " + drive.odo.getLocation()[1]);
            multTelemetry.addData("X", drive.odo.getLocation()[0]);
            multTelemetry.addData("Y", drive.odo.getLocation()[1]);
            telemetry.addData("Heading", drive.odo.getLocation()[2]);
            telemetry.addData("\t", " ");
            telemetry.addData("Veritical Encoder", drive.odo.getRawValues()[0]);
            telemetry.addData("Horizontal Encoder", drive.odo.getRawValues()[1]);
            telemetry.update();
            multTelemetry.update();
        }



    }

    private void initialize(){
        telemetry.addData("status","initialized");
        telemetry.update();
    }

}
