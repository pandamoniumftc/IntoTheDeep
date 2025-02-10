package org.firstinspires.ftc.teamcode.Util;

import static java.lang.Double.min;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.opencv.core.Point;

import java.util.ArrayList;

public class Spline {
    public Pose2d P0, P1;
    public ArrayList<Double> tValues = new ArrayList<>();
    double[] xCoefficients = new double[4], yCoefficients = new double[4];
    public Spline(Pose2d P0, double startingTheta, Pose2d P1, double endingTheta) {
        this.P0 = P0;
        this.P1 = P1;
        double velo = Math.hypot(P0.getX()-P1.getX(), P0.getY()-P1.getY());

        Translation2d lastPoint = P0.getTranslation();

        xCoefficients[0] = P0.getX();
        xCoefficients[1] = velo * cos(startingTheta);
        xCoefficients[2] = 3*P1.getX()-3*xCoefficients[0]-2*xCoefficients[1]-velo*cos(endingTheta);
        xCoefficients[3] = P1.getX() - xCoefficients[0] - xCoefficients[1] - xCoefficients[2];

        yCoefficients[0] = P0.getY();
        yCoefficients[1] = velo * sin(startingTheta);
        yCoefficients[2] = 3*P1.getY()-3*yCoefficients[0]-2*yCoefficients[1]-velo*sin(endingTheta);
        yCoefficients[3] = P1.getY() - yCoefficients[0] - yCoefficients[1] - yCoefficients[2];

        tValues.add(0.0);

        for (double time = 0.0; time < 1.0; time+=0.001) {
            Translation2d point = getPathPosition(time);
            if(lastPoint.getDistance(point) > 50.8) {
                tValues.add(time);
                lastPoint = point;
            }
        }

        tValues.add(1.0);
    }
    public Translation2d getPathPosition(double time) {
        double x = xCoefficients[0] + xCoefficients[1]*time + xCoefficients[2]*time*time + xCoefficients[3]*time*time*time;
        double y = yCoefficients[0] + yCoefficients[1]*time + yCoefficients[2]*time*time + yCoefficients[3]*time*time*time;
        return new Translation2d(x, y);
    }

    public Vector2d getTangent(double time) {
        double x = xCoefficients[1] + 2 * xCoefficients[2]*time + 3 * xCoefficients[3] * time * time;
        double y = yCoefficients[1] + 2 * yCoefficients[2]*time + 3 * yCoefficients[3] * time * time;
        return new Vector2d(x, y);
    }
}
