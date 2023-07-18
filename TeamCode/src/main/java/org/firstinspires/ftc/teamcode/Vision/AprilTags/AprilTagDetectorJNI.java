package org.firstinspires.ftc.teamcode.Vision.AprilTags;

import org.opencv.BuildConfig;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.ApriltagDetectionJNI;

import java.util.ArrayList;

public class AprilTagDetectorJNI {

    public enum TagFamily{

        TAG_36h11("tag36h11"),
        TAG_25h9("tag25h9"),
        TAG_16h5("tag16h5"),
        TAG_standard41h12("tagStandard41h12");

        public final String string;

        TagFamily(String string)
        {
            this.string = string;
        }
    }

    /**create new AprilTag detector
     * @param tagFamily - the tag family the detector will use
     * @param decimate - the decimation factor
     * @param threads - the # of threads to use
     * @return a native pointer to the newly created detector
     */
    public static native long createApriltagDetector(String tagFamily, float decimate, int threads);

    /**
     * Set the decimation parameter of a previously created AprilTag detector
     * @param ptrDetector native pointer to an AprilTag detector, obtained from {@link #createApriltagDetector(String, float, int)}
     * @param decimate the new decimation value
     */
    public static native void setApriltagDetectorDecimation(long ptrDetector, float decimate);

    /**
     * Run an AprilTag detector on a greyscale image
     * @param ptrDetector native pointer to an AprilTag detector, obtained from {@link #createApriltagDetector(String, float, int)}
     * @param ptrGreyscaleBuf native pointer to a greyscale image buffer
     * @param width the width of the greyscale buffer
     * @param height the height of the greyscale buffer
     * @return a native pointer to a list of detections, or 0 if nothing was detected.
     */
    public static native long runApriltagDetector(long ptrDetector, long ptrGreyscaleBuf, int width, int height);

    /**
     * Run an AprilTag detector on a greyscale image
     * @param ptrDetector native pointer to an AprilTag detector, obtained from {@link #createApriltagDetector(String, float, int)}
     * @param grey a greyscale OpenCV Mat
     * @param tagSize size of the tag in meters
     * @param fx lens intrinsics fx
     * @param fy lens intrinsics fy
     * @param cx lens intrinsics cx
     * @param cy lens intrinsics cy
     * @return an ArrayList of AprilTag detections. No care need be taken to release native detections.
     */
    public static ArrayList<AprilTagDetection> runAprilTagDetectorSimple(long ptrDetector, Mat grey, double tagSize, double fx, double fy, double cx, double cy)
    {
        ArrayList<org.openftc.apriltag.AprilTagDetection> detections = new ArrayList<>();
        long ptrDetectionArray = runApriltagDetector(ptrDetector, grey.dataAddr(), grey.width(), grey.height());
        if(ptrDetectionArray != 0)
        {
            detections = ApriltagDetectionJNI.getDetections(ptrDetectionArray, tagSize, fx, fy, cx, cy);
            ApriltagDetectionJNI.freeDetectionList(ptrDetectionArray);
        }

        return detections;
    }

    /**
     * Release an AprilTag detector
     * @param ptr native pointer to an AprilTag detector, obtained from {@link #createApriltagDetector(String, float, int)}
     */
    public static native void releaseApriltagDetector(long ptr);

    static
    {
        System.out.println("AprilTag plugin v" + BuildConfig.VERSION_NAME);
        System.loadLibrary("apriltag");
    }

}
