package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.firstinspires.ftc.teamcode.Util.BezierCurvePath;
import org.firstinspires.ftc.teamcode.Util.LinePath;
import org.opencv.core.Point;

public class PushSampleIntoZoneCommand extends SequentialCommandGroup {
    public PushSampleIntoZoneCommand() {
        super(
                new SplineCommand(new Pose2d(), new Point(), new Point()), // move from bar to in front of sample
                new SplineCommand(new Pose2d(), new Point(), new Point()), // pushes sample and goes in front of next
                new SplineCommand(new Pose2d(), new Point(), new Point()), // repeats
                new PositionCommand(new Pose2d()), // pushes last one
                new PositionCommand(new Pose2d()) // moves out of zone and position outside of zone, ready to pick specimen
        );
    }
}
