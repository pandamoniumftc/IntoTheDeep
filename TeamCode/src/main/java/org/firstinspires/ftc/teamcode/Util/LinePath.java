package org.firstinspires.ftc.teamcode.Util;

import static java.lang.Math.pow;

public class LinePath {
    public Pose P0, P1;
    public LinePath(Pose P0, Pose P1) {
        this.P0 = P0;
        this.P1 = P1;
    }
    public Pose returnPosition(double t) {
        double x = (1-t) * P0.x + t * P1.x;
        double y = (1-t) * P0.y + t * P1.y;
        return new Pose(x, y);
    }
    public Pose returnVelocity() {
        double x = P1.x - P0.x;
        double y = P1.y - P0.y;
        return new Pose(x, y);
    }
    public boolean isPositionOnPath() {
        return true;
    }
}
