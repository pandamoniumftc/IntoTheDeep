package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.opencv.core.Point;

public class PushSampleIntoZoneCommand extends SequentialCommandGroup {
    public PushSampleIntoZoneCommand() {
        super(
                new PositionCommand(new Pose2d(1050, 654, new Rotation2d(0)), 0.8),
                new PositionCommand(new Pose2d(1050, 1176, new Rotation2d(0)), 0.8),
                new PositionCommand(new Pose2d(1400, 1176, new Rotation2d(0)), 0.8),
                //new SplineCommand(new Pose2d(1400, 1176, new Rotation2d(0)), Math.toRadians(90), Math.toRadians(-90), 0.6), // move from bar to in front of sample
                new PositionCommand(new Pose2d(1400, 350, new Rotation2d(0)), 0.8), // pushes sample and goes in front of next
                new PositionCommand(new Pose2d(1400, 700, new Rotation2d(0)), 0.8)
                //new PositionCommand(new Pose2d(1800, 1176, new Rotation2d(0)), 0.8),
                //new PositionCommand(new Pose2d(1700, 350, new Rotation2d(0)), 0.8),
                //new PositionCommand(new Pose2d(1700, 700, new Rotation2d(0)), 0.8)
                //new SplineCommand(new Pose2d(1800, 1176, new Rotation2d(0)), Math.toRadians(90), Math.toRadians(-90), 0.6) // repeats
                //new PositionCommand(new Pose2d(), 0.5), // pushes last one
                //new PositionCommand(new Pose2d(), 0.5) // moves out of zone and position outside of zone, ready to pick specimen
        );
    }
}
