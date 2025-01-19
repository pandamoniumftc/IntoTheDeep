package org.firstinspires.ftc.teamcode.Subsystem.CameraSystems;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.opencv.core.Core;
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
import java.util.List;

public class SampleAlignmentPipeline extends OpenCvPipeline {
    PandaRobot robot = PandaRobot.getInstance();
    Mat dst = new Mat();
    private Point center = new Point();
    private double sampleAngle = 0.0;
    boolean viewportPaused;
    Scalar yellowLower = new Scalar(60, 120, 0);
    Scalar yellowUpper = new Scalar(100, 255, 255);
    Scalar blueLower = new Scalar(0, 140, 0);
    Scalar blueUpper = new Scalar(255, 255, 200);
    Scalar redLower = new Scalar(0, 0, 180);
    Scalar redUpper = new Scalar(255, 140, 255);
    /*Scalar yellowLower = new Scalar(60, 200, 100);
    Scalar yellowUpper = new Scalar(120, 255, 255);
    Scalar blueLower = new Scalar(0, 140, 0);
    Scalar blueUpper = new Scalar(255, 255, 200);
    Scalar redLower = new Scalar(0, 0, 180);
    Scalar redUpper = new Scalar(255, 140, 255);*/
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.resize(input, input, new Size(80, Math.round((80.0 / input.size().width) * input.size().height)));

        Mat color = new Mat();
        Mat yellow = new Mat();
        Imgproc.cvtColor(input, color, Imgproc.COLOR_BGR2YCrCb);
        Imgproc.cvtColor(input, yellow, Imgproc.COLOR_BGR2HSV);

        Core.inRange(yellow, yellowLower, yellowUpper, yellow);

        if (Globals.alliance == Globals.RobotAlliance.BLUE) {
            Core.inRange(color, blueLower, blueUpper, color);
        } else {
            Core.inRange(color, redLower, redUpper, color);
        }

        Core.bitwise_or(color, yellow, dst);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(dst, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        contours.removeIf(contour -> Imgproc.contourArea(contour) < 100.0);

        double minDistance = Double.MAX_VALUE;
        MatOfPoint closestSample = null;

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approxCurve, 0.02 * Imgproc.arcLength(contour2f, true), true);

            Moments moments = Imgproc.moments(contour);
            double contourCenterX = moments.get_m10() / moments.get_m00();
            double contourCenterY = moments.get_m01() / moments.get_m00();
            double dx = contourCenterX - input.width() / 2.0;
            double dy = contourCenterY - input.height() / 2.0;
            double distance = dx * dx + dy * dy;

            if (approxCurve.total() >= 4 && approxCurve.total() <= 6 && distance < minDistance) {
                minDistance = distance;
                closestSample = new MatOfPoint(approxCurve.toArray());
            }
        }

        if (closestSample != null) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(closestSample.toArray()));
            sampleAngle = calculateAngle(rect);
            Point[] points = new Point[4];
            rect.points(points);

            center = rect.center;

            for (int i = 0; i < points.length; i++) {
                Imgproc.line(input, points[i], points[(i + 1) % points.length], new Scalar(0, 0, 255));
            }

            //Imgproc.putText(input, String.valueOf(sampleAngle), center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255));

            Imgproc.putText(input, center.toString(), center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.20, new Scalar(255, 255, 255));
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
    public Point getSamplePosition() {
        return center;
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
            robot.oldAssCam.pauseViewport();
        }
        else
        {
            robot.oldAssCam.resumeViewport();
        }
    }
}
