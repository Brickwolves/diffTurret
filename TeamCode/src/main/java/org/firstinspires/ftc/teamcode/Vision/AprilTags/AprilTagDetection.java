package org.firstinspires.ftc.teamcode.Vision.AprilTags;

import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagPose;

public class AprilTagDetection {

    public int id;
    public int hamming;
    public float decisionMargin;
    public Point center;
    public Point[] corners;
    public AprilTagPose pose;

}
