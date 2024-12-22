package org.firstinspires.ftc.teamcode.Subsystem.CameraSystems;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class SampleAlignmentPipeline extends OpenCvPipeline {
    Robot robot = Robot.getInstance();
    Mat dst = new Mat();
    private double sampleAngle = 0.0;
    boolean viewportPaused;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.resize(input, input, new Size(80, (int) Math.round((80 / input.size().width) * input.size().height)));

        Mat color = input.clone();
        Mat yellow = input.clone();

        Imgproc.cvtColor(color, color, Imgproc.COLOR_BGR2YCrCb);
        Imgproc.cvtColor(yellow, yellow, Imgproc.COLOR_BGR2HSV);

        Core.inRange(yellow, new Scalar(60, 190, 100), new Scalar(120, 255, 255), yellow);

        if (Globals.alliance == Globals.RobotAlliance.BLUE) {
            Core.inRange(color, new Scalar(0, 130, 0), new Scalar(255, 255, 200), color);
        }
        else {
            Core.inRange(color,new Scalar(0, 0, 180), new Scalar(255, 140, 255), color);
        }

        Core.bitwise_or(color, yellow, dst);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(dst, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double minArea = 200.0; // Minimum area threshold
        double minDistance = Double.MAX_VALUE;
        MatOfPoint closestSample = null;

        for (MatOfPoint contour : contours) {
            // Approximate the contour to a polygon
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approxCurve, 0.02 * Imgproc.arcLength(contour2f, true), true);

            Moments moments = Imgproc.moments(contour);

            // Calculate the center of the contour
            double contourCenterX = moments.get_m10() / moments.get_m00();
            double contourCenterY = moments.get_m01() / moments.get_m00();
            Point contourCenter = new Point(contourCenterX, contourCenterY);

            // Calculate the distance to the image center
            double distance = Math.sqrt(Math.pow(contourCenter.x - (input.width() / 2.0), 2) +
                    Math.pow(contourCenter.y, 2));

            // Check if the approximated polygon has 4 sides and meets the area requirement
            if ((approxCurve.total() >= 4 || approxCurve.total() <= 6) && Imgproc.contourArea(contour) > minArea && distance < minDistance) {
                minDistance = distance;
                closestSample = new MatOfPoint(approxCurve.toArray());
            }
        }

        if (closestSample != null) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(closestSample.toArray()));
            sampleAngle = calculateAngle(rect);
            Point[] points = new Point[4];
            rect.points(points);
            for(int i=0; i<4; ++i){
                Imgproc.line(input, points[i], points[(i+1)%4], new Scalar(0,0,255));
            }
            Imgproc.putText(input, String.valueOf(sampleAngle), rect.center, 3, 0.5, new Scalar(255, 255, 255));
            Imgproc.line(input, rect.center, new Point((points[2].x + points[3].x)/2, (points[2].y + points[3].y)/2), new Scalar(255,0,0));
        }

        return input;
    }

    private double calculateAngle(RotatedRect rotatedRect) {
        if (rotatedRect.size.width>rotatedRect.size.height) {
            return rotatedRect.angle + 90.0;
        }
        else {
            return rotatedRect.angle;
        }
    }

    public double getSampleAngle() {
        return sampleAngle;
    }
    @Override
    public void onViewportTapped() {
        /*
         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
         * when you need your vision pipeline running, but do not require a live preview on the
         * robot controller screen. For instance, this could be useful if you wish to see the live
         * camera preview as you are initializing your robot, but you no longer require the live
         * preview after you have finished your initialization process; pausing the viewport does
         * not stop running your pipeline.
         *
         * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
         */

        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            robot.logitechCam.pauseViewport();
        }
        else
        {
            robot.logitechCam.resumeViewport();
        }
    }
}
