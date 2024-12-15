package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BrentSolver;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public class LinePath {
    public Pose2d P0, P1;
    public Vector2d pathVector;
    public enum HeadingBehavior {
        STATIC,
        LINEAR,
        FOLLOW
    }
    public HeadingBehavior headingBehavior;
    private BrentSolver solver;
    public LinePath(Pose2d P0, Pose2d P1, HeadingBehavior behavior) {
        this.P0 = P0;
        this.P1 = P1;
        headingBehavior = behavior;
        pathVector = new Vector2d(P1.getX() - P0.getX(), P1.getY() - P0.getY());
        solver = new BrentSolver();
    }

    // returns x and y position based on parameter t
    public Vector2d getPathPosition(double t) {
        double x = (1-t) * P0.getX() + t * P1.getX();
        double y = (1-t) * P0.getY() + t * P1.getY();
        return new Vector2d(x, y);
    }

    // gives magnitude (velo) and direction (angle of path)
    public Vector2d returnTangentVector() {
        return pathVector;
    }

    // returns heading based on parameter t
    public double getHeading(double t) {
        switch (headingBehavior) {
            case STATIC:
                return t == 1.0 ? P0.getHeading() : P1.getHeading();
            case LINEAR:
                return t * P1.getHeading() + (1 - t) * P0.getHeading();
            case FOLLOW:
                return t == 1.0 ? P1.getHeading() : normalizeRadians(pathVector.angle());
        }
        return 0;
    }

    // get closest position to robot's current position
    public Vector2d getClosestPosition(Pose2d current) {
        UnivariateFunction distanceSquared = t -> {
            double x = getPathPosition(t).getX();
            double y = getPathPosition(t).getY();
            return Math.pow(x - current.getX(), 2) + Math.pow(y - current.getY(), 2);
        };
        double t = solver.solve(1000, distanceSquared, 0.0, 1.0);
        return getPathPosition(t);
    }
}
