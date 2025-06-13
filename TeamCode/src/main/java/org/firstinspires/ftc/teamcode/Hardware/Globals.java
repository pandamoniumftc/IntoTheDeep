package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;

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
    public static boolean targetingColorPiece = true;
}
