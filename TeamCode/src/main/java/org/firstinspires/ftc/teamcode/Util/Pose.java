package org.firstinspires.ftc.teamcode.Util;

import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

public class Pose {
    public double x, y, r, px, py, pr;
    public Pose(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;

        this.px = 0;
        this.py = 0;
        this.pr = 0;
    }
    public Pose(double x, double y) {
        this.x = x;
        this.y = y;
        this.r = 0;

        this.px = 0;
        this.py = 0;
        this.pr = 0;
    }
    public void setPose(Pose p) {
        x = p.x;
        y = p.y;
        r = p.r;
    }
    public void setPrevPose(Pose p) {
        px = p.x;
        py = p.y;
        pr = p.r;
    }
    public void add(Pose p) {
        x += p.x;
        y += p.y;
        r += p.r;
    }
    public double getXYLength() {
        return hypot(x, y);
    }
    public void normalize() {
        double len = getXYLength();
        if (len == 0) len = 1E-6;
        x /= len;
        y /= len;
    }

    public void rotate(double angle, Pose pivot) {
        Pose t = new Pose(x - pivot.x, y - pivot.y, r);
        double len = getXYLength();
        if (len == 0) len = 1E-6;
        t.normalize();
        x = (t.x * cos(angle) - t.y * sin(angle)) * len + pivot.x;
        y = (t.y * cos(angle) + t.x * sin(angle)) * len + pivot.y;
    }
}
