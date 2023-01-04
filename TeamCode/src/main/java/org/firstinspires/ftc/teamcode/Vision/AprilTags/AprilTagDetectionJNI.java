package org.firstinspires.ftc.teamcode.Vision.AprilTags;

import org.opencv.core.Point;

import java.util.ArrayList;

public class AprilTagDetectionJNI {


    // Get the tag ID of a detection
    public static native int getId(long ptr);

    //get the hamming of a detection
    public static native int getHamming(long ptr);

    //get the decision margin of a detection
    public static native float getDecisionMargin(long ptr);

    //get the enterpoint of a detection
    public static native double[] getCenterpoint(long ptr);

    //get the corners of a detection
    public static native double[][] getCorners(long ptr);

    //get the pose estimate for a detection
    public static native double[] getPoseEstimate(long ptr, double tagSize, double fx, double fy, double cx, double cy);

    //get a pointer for each of the detections inside a list returned by AprilTagDetectorJNI(long, long, int, int)
    public static native long[] getDetectionPointers(long ptrZarray);

    //frees a list of detections obtained from AprilTagDetectorJNI(long, long, int, int)
    public static native void freeDetectionList(long ptrDetections);

    //creates a decoupled java representation of the detections in the native detection list
    public static ArrayList<AprilTagDetection> getDetections(long ptrDetections, double tagSize, double fx, double fy, double cx, double cy)
    {
        long[] detectionPointers = getDetectionPointers(ptrDetections);
        ArrayList<AprilTagDetection> detections = new ArrayList<>(detectionPointers.length);

        for(long ptrDetection : detectionPointers)
        {
            AprilTagDetection detection = new AprilTagDetection();
            detection.id = getId(ptrDetection);
            detection.hamming = getHamming(ptrDetection);
            detection.decisionMargin = getDecisionMargin(ptrDetection);
            double[] center = getCenterpoint(ptrDetection);
            detection.center = new Point(center[0], center[1]);
            double[][] corners = getCorners(ptrDetection);

            detection.corners = new Point[4];
            for(int p = 0; p < 4; p++)
            {
                detection.corners[p] = new Point(corners[p][0], corners[p][1]);
            }

            detection.pose = new org.openftc.apriltag.AprilTagPose();
            double[] pose = getPoseEstimate(ptrDetection, tagSize, fx, fy, cx, cy);
            detection.pose.x = pose[0];
            detection.pose.y = pose[1];
            detection.pose.z = pose[2];
            detection.pose.yaw = pose[3];
            detection.pose.pitch = pose[4];
            detection.pose.roll = pose[5];

            detections.add(detection);
        }

        return detections;
    }

    static
    {
        System.loadLibrary("apriltag");
    }

}
