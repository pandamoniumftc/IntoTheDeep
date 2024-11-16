package org.firstinspires.ftc.teamcode.Util;

import static com.qualcomm.robotcore.util.Range.scale;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

public class Pose {
    public double x, y, r;
    public enum Operation {
        ADD,
        SUBTRACT,
        MULTIPLY,
        DIVIDE
    }
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
    public void applyOperation(Pose p, Operation opp) {
        switch (opp) {
            case ADD:
                x += p.x;
                y += p.y;
                r += p.r;
                break;
            case SUBTRACT:
                x -= p.x;
                y -= p.y;
                r -= p.r;
                break;
            case MULTIPLY:
                x *= p.x;
                y *= p.y;
                r *= p.r;
                break;
            case DIVIDE:
                x /= p.x;
                y /= p.y;
                r /= p.r;
                break;
        }
    }

    public double magnitude() {
        return hypot(x, y);
    }
    public void normalize() {
        double len = magnitude();
        if (len == 0) len = 1E-6;
        x /= len;
        y /= len;
    }
    public void rotate(double angle) {
        double len = magnitude();
        if (len == 0) len = 1E-6;

        Pose n = new Pose(x, y, r);
        n.normalize();

        x = (n.x * cos(angle) - n.y * sin(angle)) * len;
        y = (n.y * cos(angle) + n.x * sin(angle)) * len;
    }
}
