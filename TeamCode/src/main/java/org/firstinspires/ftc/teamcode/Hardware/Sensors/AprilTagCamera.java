package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Diagnostics.RGBDiagnostic;
import org.firstinspires.ftc.teamcode.Vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.Vision.TurretPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class AprilTagCamera {

    private OpenCvCamera webcam;
    private String id;
    public AprilTagPipeline pipeline = new AprilTagPipeline(0.04, 1399.629,1399.22,653.019,364.454);


    public AprilTagCamera(String name){
        id = name;


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, name), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }

            public void close(){
                webcam.closeCameraDevice();
            }

        });

    }
}

