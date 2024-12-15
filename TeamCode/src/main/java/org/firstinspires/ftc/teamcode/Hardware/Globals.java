package org.firstinspires.ftc.teamcode.Hardware;

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
    public static boolean telemetryEnable = false;
}
