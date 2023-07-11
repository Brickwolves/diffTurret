package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.DashConstants.VisionDiagnostic.DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.DashConstants.dashVision.blur;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utilities.VisionUtils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class TurretPipeline extends OpenCvPipeline {
    private Mat output = new Mat();
    private  Mat modifiedYellow = new Mat();
    private Rect target = new Rect();


    List<MatOfPoint> yellowContours = new ArrayList<>();
    Mat yellowHierarchy = new Mat();
    Scalar green = new Scalar(0,255,0);


    @Override
    public Mat processFrame(Mat input){


        input.copyTo(output);
        input.copyTo(modifiedYellow);
        cvtColor(modifiedYellow,modifiedYellow,COLOR_RGB2HSV);

        Scalar minYellowHSV = new Scalar(20,100,100);
        Scalar maxYellowHSV = new Scalar(30,255,255);

        //TODO DELETE MAYBE
        GaussianBlur(modifiedYellow,modifiedYellow,new Size(blur,blur),0);

        inRange(modifiedYellow,minYellowHSV,maxYellowHSV,modifiedYellow);


        yellowContours = new ArrayList<>();

        findContours(modifiedYellow,yellowContours,yellowHierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);

        List<Rect> yellowRects = new ArrayList<>();

        for(int cnt_index = 0; cnt_index < yellowContours.size(); cnt_index++){
            drawContours(output,yellowContours,cnt_index,green,2);

            Rect rect = boundingRect(yellowContours.get(cnt_index));

            yellowRects.add(rect);






        }



        if(yellowRects.size()==0){
            return output;
        }

        Rect largestYellowRect = VisionUtils.sortRectsByMaxOption(1,VisionUtils.RECT_OPTION.AREA,yellowRects).get(0);
        target = largestYellowRect;
        rectangle(output,largestYellowRect,green,2);



        if(DEBUG_MODE){
            return modifiedYellow;
        }
        return output;
    }

    public double getTargetX(){
        return target.x;
    }

    public double getTargetY(){
        return target.y;
    }

    public double checkSize(){return target.width;}

}
