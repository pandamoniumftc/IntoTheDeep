package org.firstinspires.ftc.teamcode.Util;

public class SplinePose extends Pose2d {
    public double radius, power;
    public SplinePose(Pose2d pose, double radius, double power) {
        super(pose.x, pose.y, pose.heading);
        this.radius = radius;
        this.power = power;
    }

    public SplinePose() {
        super(0, 0, 0);
        this.radius = 0;
        this.power = 0;
    }
}
