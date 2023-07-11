package org.firstinspires.ftc.teamcode.Diagnostics;
import static org.opencv.imgproc.Imgproc.cvtColor;

import org.opencv.core.Mat;

import org.opencv.core.Scalar;

import org.openftc.easyopencv.OpenCvPipeline;
public class RGBDiagnostic extends OpenCvPipeline {

    public Mat m = new Mat();

    public double[] findPoint() {
        return m.get(160,120);

    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(m);
        return m;
    }
}
