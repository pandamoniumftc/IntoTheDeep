package org.firstinspires.ftc.teamcode.Util;

import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
    private BrentSolver solver;
    public BezierCurvePath(Pose2d P0, Point P1, Point P2, Pose2d P3, HeadingBehavior behavior) {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;
        headingBehavior = behavior;
        solver = new BrentSolver();
    }
    public Vector2d getPosition(double t) {
        double x = pow(1-t, 3) * P0.getX() + 3 * pow(1-t, 2) * t * P1.x + 3 * (1-t) * pow(t, 2) * P2.x + pow(t, 3) * P3.getX();
        double y = pow(1-t, 3) * P0.getY() + 3 * pow(1-t, 2) * t * P1.y + 3 * (1-t) * pow(t, 2) * P2.y + pow(t, 3) * P3.getY();
        return new Vector2d(x, y);
    }
    public Vector2d getFirstDerivative(double t) {
        double x = -3 * pow(1-t, 2) * P0.getX() + 3 * (1-t) * (1-2*t) * P1.x + 3 * t * (2-t) * P2.x + 3 * pow(t, 2) * P3.getX();
        double y = -3 * pow(1-t, 2) * P0.getY() + 3 * (1-t) * (1-2*t) * P1.y + 3 * t * (2-t) * P2.y + 3 * pow(t, 2) * P3.getY();
        return new Vector2d(x, y);
    }
    public Vector2d getSecondDerivative(double t) {
        double x = 6 * (1-t) * P0.getX() + 6 * (1-3*t) * P1.x + 6 * (1-t) * P2.x + 6 * t * P3.getX();
        double y = 6 * (1-t) * P0.getY() + 6 * (1-3*t) * P1.y + 6 * (1-t) * P2.y + 6 * t * P3.getY();
        return new Vector2d(x, y);
    }
    /*
        ( 1 + (dy/dx)^2 ) ^ (3/2)
    r = ---------------------------
               (d^2y/dx^2)
     */
    public double getRadiusCurvature(double t) {
        return pow(sqrt(1 + pow(getFirstDerivative(t).magnitude(), 2)), 3) / getSecondDerivative(t).magnitude();
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
            return Math.pow(x - current.getX(), 2) + Math.pow(y - current.getY(), 2);
        };
        double t = solver.solve(1000, distanceSquared, 0.0, 1.0);
        return getPosition(t);
    }
}
