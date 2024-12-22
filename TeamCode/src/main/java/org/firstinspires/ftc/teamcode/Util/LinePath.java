package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.hypot;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public class LinePath {
    public Pose2d P0, P1;
    public Vector2d pathVector;
    private BrentOptimizer solver;
    public LinePath(Pose2d P0, Pose2d P1) {
        this.P0 = P0;
        this.P1 = P1;
        pathVector = new Vector2d(P1.getX() - P0.getX(), P1.getY() - P0.getY());
        //solver = new BrentOptimizer(1e-5, 1e-10);
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
        return t * P1.getHeading() + (1 - t) * P0.getHeading();
    }

    // get closest position to robot's current position
    /*public Vector2d getClosestPosition(Pose2d current, double currT) {
        UnivariateFunction distanceSquared = t -> {
            double x = getPathPosition(t).getX();
            double y = getPathPosition(t).getY();
            return hypot(x - current.getX(), y - current.getY());
        };
        double t = solver.optimize(GoalType.MINIMIZE, new UnivariateObjectiveFunction(distanceSquared), new SearchInterval(0.0, 1.0)).getPoint();
        return getPathPosition(t);
    }*/
}
