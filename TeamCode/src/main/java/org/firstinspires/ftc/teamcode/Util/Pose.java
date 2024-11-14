package org.firstinspires.ftc.teamcode.Util;

import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

public class Pose {
    public double x, y, r;
    public Pose() {
        this.x = 0;
        this.y = 0;
        this.r = 0;
    }
    public Pose(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }
    public Pose(double x, double y) {
        this.x = x;
        this.y = y;
        this.r = 0;
    }
    public void setPose(Pose p) {
        x = p.x;
        y = p.y;
        r = p.r;
    }
    public void add(Pose p) {
        x += p.x;
        y += p.y;
        r += p.r;
    }
    public void subtract(Pose p) {
        x -= p.x;
        y -= p.y;
        r -= p.r;
    }
    public double magnitude() {
        return hypot(x, y);
    }
    public void rotate(double angle) {
        x = x * cos(angle) - y * sin(angle);
        y = y * cos(angle) + x * sin(angle);
    }
}
