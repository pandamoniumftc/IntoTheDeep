package org.firstinspires.ftc.teamcode.Util;

import static java.lang.Math.pow;

public class BezierCurvePath {
    public Pose P0, P1, P2, P3;
    public Pose returnPosition(double t) {
        double x = pow(1-t, 3) * P0.x + 3 * pow(1-t, 2) * t * P1.x + 3 * (1-t) * pow(t, 2) * P2.x + pow(t, 3) * P3.x;
        double y = pow(1-t, 3) * P0.y + 3 * pow(1-t, 2) * t * P1.y + 3 * (1-t) * pow(t, 2) * P2.y + pow(t, 3) * P3.y;
        return new Pose(x, y);
    }
    public Pose returnVelocity(double t) {
        double x = -3 * pow(1-t, 2) * P0.x + 3 * (1-t) * (1-2*t) * P1.x + 3 * t * (2-t) * P2.x + 3 * pow(t, 2) * P3.x;
        double y = -3 * pow(1-t, 2) * P0.y + 3 * (1-t) * (1-2*t) * P1.y + 3 * t * (2-t) * P2.y + 3 * pow(t, 2) * P3.y;
        return new Pose(x, y);
    }
    public Pose returnAcceleration(double t) {
        double x = 6 * (1-t) * P0.x + 6 * (1-3*t) * P1.x + 6 * (1-t) * P2.x + 6 * t * P3.x;
        double y = 6 * (1-t) * P0.y + 6 * (1-3*t) * P1.y + 6 * (1-t) * P2.y + 6 * t * P3.y;
        return new Pose(x, y);
    }
}
