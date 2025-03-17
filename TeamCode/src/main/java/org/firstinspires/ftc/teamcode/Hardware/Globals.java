package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class Globals {
    public enum RobotAlliance {
        BLUE,
        RED
    }
    public static RobotAlliance alliance;
    public enum RobotOpMode {
        AUTO,
        TELEOP
    }
    public static RobotOpMode opMode;
    public static Pose2d pose;
}
