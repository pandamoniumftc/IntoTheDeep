package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import java.util.ArrayList;

public class Spline {
    public ArrayList<SplinePose> poses = new ArrayList<>();
    double[] xCoefficients = new double[4], yCoefficients = new double[4];
    public double[] xPoints = new double[100], yPoints = new double[100];
    /**
     * @param startingPose starting position of the spline
     */
    public Spline(Pose2d startingPose) {
        poses.add(new SplinePose(startingPose, 100.0, 1.0));
    }

    public Spline addPoint(Pose2d pose, double maxPower) {
        SplinePose lastPose = poses.get(poses.size() - 1);

        double velo = Math.hypot(lastPose.x-pose.x, lastPose.x-pose.x);

        xCoefficients[0] = lastPose.x;
        xCoefficients[1] = velo * cos(lastPose.heading);
        xCoefficients[2] = 3*pose.x-3* xCoefficients[0]-2* xCoefficients[1]-velo*cos(Math.toRadians(pose.heading));
        xCoefficients[3] = pose.x - xCoefficients[0] - xCoefficients[1] - xCoefficients[2];

        yCoefficients[0] = lastPose.y;
        yCoefficients[1] = velo * sin(Math.toRadians(lastPose.heading));
        yCoefficients[2] = 3*pose.y-3* yCoefficients[0]-2* yCoefficients[1]-velo*sin(Math.toRadians(pose.heading));
        yCoefficients[3] = pose.y - yCoefficients[0] - yCoefficients[1] - yCoefficients[2];

        poses.add(lastPose);

        Pose2d lastPoint = lastPose.clone();

        for (double time = 0.0; time < 1.0; time+=0.001) {
            Pose2d point = position(time);
            if(lastPoint.getDistanceFromPoint(point) > 100) {
                double[] v = velocity(time);
                point.heading = atan2(v[1], v[0]);
                poses.add(new SplinePose(point, findRadius(time), maxPower));
                lastPoint = point;
            }
        }

        poses.add(new SplinePose(pose, findRadius(1.0), maxPower));

        int startingIndex = poses.size() - 1 == -1 ? 0 : poses.size() - 1;

        for (int i = startingIndex; i < poses.size(); i++) {
            Pose2d p = poses.get(i);
            xPoints[i] = p.x / 25.4;
            yPoints[i] = p.y / 25.4;
        }

        return this;
    }

    public Pose2d position(double time) {
        double x = xCoefficients[0] + xCoefficients[1]*time + xCoefficients[2]*time*time + xCoefficients[3]*time*time*time;
        double y = yCoefficients[0] + yCoefficients[1]*time + yCoefficients[2]*time*time + yCoefficients[3]*time*time*time;
        return new Pose2d(x, y);
    }

    public double[] velocity(double time) {
        double x = xCoefficients[1] + 2* xCoefficients[2]*time + 3 * xCoefficients[3]*time*time;
        double y = yCoefficients[1] + 2* yCoefficients[2]*time + 3* yCoefficients[3]*time*time;
        return new double[] {x, y};
    }

    public double[] accel(double time) {
        double x = 2* xCoefficients[2] + 6* xCoefficients[3]*time;
        double y = 2* yCoefficients[2] + 6* yCoefficients[3]*time;
        return new double[] {x, y};
    }

    public double findRadius(double time){
        double[] velocity = velocity(time);
        double[] accel = accel(time);

        if ((accel[1] * velocity[0] - accel[0] * velocity[1]) != 0) {
            double r = Math.pow(velocity[0] * velocity[0] + velocity[1] * velocity[1], 1.5) / (accel[1] * velocity[0] - accel[0] * velocity[1]);
            return Math.min(Math.abs(r),2540) * signum(r);
        }
        return 2540;
    }
}
