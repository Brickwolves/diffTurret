package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;


public class JunctionPipeline extends OpenCvPipeline {
//this pipeline is for auto-aligning the dropper over the junction
//to be used for teleOp (and auto?)

        @Override
        public Mat processFrame(Mat input) {
            return input;
        } //TEMPORARY

}


