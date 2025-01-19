package org.firstinspires.ftc.teamcode.Util;

import static java.lang.Double.min;
import static java.lang.Math.hypot;
import static java.lang.Math.pow;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.optim.OptimizationData;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import org.opencv.core.Point;

public class BezierCurvePath {
    /* TODO:
    - double check first derivative and second derivatives
    - make sure this class has all of the functions similar to LinePath
    - NOTE: radius curvature will be used to add correction power to account for centripetal force
     */
    public Pose2d P0, P3;
    public Point P1, P2;
    private double t = 0.0, stepValue;
    public BezierCurvePath(Pose2d P0, Point P1, Point P2, Pose2d P3, double stepValue) {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;
        this.stepValue = stepValue;
    }
    public void update() {
        t += min(stepValue, 1.0);
    }
    public Vector2d getPathPosition() {
        double x = pow(1-t, 3) * P0.getX() + 3 * pow(1-t, 2) * t * P1.x + 3 * (1-t) * pow(t, 2) * P2.x + pow(t, 3) * P3.getX();
        double y = pow(1-t, 3) * P0.getY() + 3 * pow(1-t, 2) * t * P1.y + 3 * (1-t) * pow(t, 2) * P2.y + pow(t, 3) * P3.getY();
        return new Vector2d(x, y);
    }
    public Vector2d getFirstDerivative() {
        double x = 3 * Math.pow(1 - t, 2) * (P1.x - P0.getX())
                + 6 * (1 - t) * t * (P2.x - P1.x)
                + 3 * Math.pow(t, 2) * (P3.getX() - P2.x);
        double y = 3 * Math.pow(1 - t, 2) * (P1.y - P0.getY())
                + 6 * (1 - t) * t * (P2.y - P1.y)
                + 3 * Math.pow(t, 2) * (P3.getY() - P2.y);
        return new Vector2d(x, y);
    }
    public Vector2d getSecondDerivative() {
        double x = 6 * (1 - t) * (P2.x - 2 * P1.x + P0.getX())
                + 6 * t * (P3.getX() - 2 * P2.x + P1.x);
        double y = 6 * (1 - t) * (P2.y - 2 * P1.y + P0.getY())
                + 6 * t * (P3.getY() - 2 * P2.y + P1.y);
        return new Vector2d(x, y);
    }
    public double getRadiusCurvature() {
        return (pow(getFirstDerivative().magnitude(), 3)) / (getFirstDerivative().getX() * getSecondDerivative().getY() - getFirstDerivative().getY() * getSecondDerivative().getX());
    }
    public double getClosestT(Vector2d current) {
        double closestT = 0.0;
        double minDistance = Double.MAX_VALUE;

        for (double t = this.t; t <= 1.0; t += stepValue) {
            Vector2d currentPoint = getPathPosition();
            double currentDistance = currentPoint.minus(current).magnitude();

            // Update closest point if the distance is smaller
            if (currentDistance < minDistance) {
                minDistance = currentDistance;
                closestT = t;
            } else {
                // Stop iterating when the distance starts increasing
                break;
            }
        }

        return closestT;
    }
}
