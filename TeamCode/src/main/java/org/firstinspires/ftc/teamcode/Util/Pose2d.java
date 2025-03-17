package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import androidx.annotation.NonNull;

public class Pose2d implements Cloneable {
    public double x;
    public double y;
    public double heading;
    public Pose2d() {
        this(0, 0, 0);
    }
    public Pose2d(double x, double y){
        this(x,y,0);
    }
    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = normalizeRadians(heading);
    }
    public double getDistanceFromPoint(Pose2d newPoint) {
        return Math.hypot(newPoint.x - this.x, newPoint.y - this.y);
    }
    @NonNull
    public Pose2d clone() {
        return new Pose2d(x, y, heading);
    }

    @NonNull
    public String toString() {
        return "X: " + x + ", Y: " + y + ", H: " + heading;
    }
}
