package org.firstinspires.ftc.teamcode.Util;

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
    public enum HeadingBehavior {
        STATIC,
        LINEAR,
        FOLLOW
    }
    public HeadingBehavior headingBehavior;
    private BrentOptimizer solver;
    public BezierCurvePath(Pose2d P0, Point P1, Point P2, Pose2d P3, HeadingBehavior behavior) {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;
        headingBehavior = behavior;
        solver = new BrentOptimizer(1e-5, 1e-10);
    }
    public Vector2d getPosition(double t) {
        double x = pow(1-t, 3) * P0.getX() + 3 * pow(1-t, 2) * t * P1.x + 3 * (1-t) * pow(t, 2) * P2.x + pow(t, 3) * P3.getX();
        double y = pow(1-t, 3) * P0.getY() + 3 * pow(1-t, 2) * t * P1.y + 3 * (1-t) * pow(t, 2) * P2.y + pow(t, 3) * P3.getY();
        return new Vector2d(x, y);
    }
    public Vector2d getFirstDerivative(double t) {
        double x = 3 * Math.pow(1 - t, 2) * (P1.x - P0.getX())
                + 6 * (1 - t) * t * (P2.x - P1.x)
                + 3 * Math.pow(t, 2) * (P3.getX() - P2.x);
        double y = 3 * Math.pow(1 - t, 2) * (P1.y - P0.getY())
                + 6 * (1 - t) * t * (P2.y - P1.y)
                + 3 * Math.pow(t, 2) * (P3.getY() - P2.y);
        return new Vector2d(x, y);
    }
    public Vector2d getSecondDerivative(double t) {
        double x = 6 * (1 - t) * (P2.x - 2 * P1.x + P0.getX())
                + 6 * t * (P3.getX() - 2 * P2.x + P1.x);
        double y = 6 * (1 - t) * (P2.y - 2 * P1.y + P0.getY())
                + 6 * t * (P3.getY() - 2 * P2.y + P1.y);
        return new Vector2d(x, y);
    }
    public double getRadiusCurvature(double t) {
        return (pow(getFirstDerivative(t).magnitude(), 3)) / (getFirstDerivative(t).getX() * getSecondDerivative(t).getY() - getFirstDerivative(t).getY() * getSecondDerivative(t).getX());
    }
    public double getHeading(double t) {
        switch (headingBehavior) {
            case STATIC:
                return t == 1.0 ? P0.getHeading() : P3.getHeading();
            case LINEAR:
                return t * P3.getHeading() + (1 - t) * P0.getHeading();
            case FOLLOW:
                return t == 1.0 ? getFirstDerivative(t).angle() : P3.getHeading();
        }
        return 0;
    }
    public Vector2d getClosestPosition(Pose2d current) {
        UnivariateFunction distanceSquared = t -> {
            double x = getPosition(t).getX();
            double y = getPosition(t).getY();
            return hypot(x - current.getX(), y - current.getY());
        };
        double t = solver.optimize(GoalType.MINIMIZE, new UnivariateObjectiveFunction(distanceSquared), new SearchInterval(0.0, 1.0)).getPoint();
        return getPosition(t);
    }
}
