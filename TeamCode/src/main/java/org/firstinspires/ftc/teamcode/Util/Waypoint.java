package org.firstinspires.ftc.teamcode.Util;

public class Waypoint {
    public Pose2d position;
    public double moveSpeed, followDistance;

    public Waypoint(Pose2d position, double moveSpeed, double followDistance) {
        this.position = position;
        this.moveSpeed = moveSpeed;
        this.followDistance = followDistance;
    }

    public Waypoint(Waypoint thisPoint) {
        this.position = thisPoint.position;
        this.moveSpeed = thisPoint.moveSpeed;
        this.followDistance = thisPoint.followDistance;
    }

    public void setPoint(Pose2d pose) {
        this.position = pose;
    }
}
