package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class CycleSpecimenCommand extends SequentialCommandGroup {
    public CycleSpecimenCommand(double specimenNum) {
        super(
                new PositionCommand(new Pose2d(1250, 169, new Rotation2d(Math.toRadians(180))), 0.6),
                new PositionCommand(new Pose2d(1250, 109, new Rotation2d(Math.toRadians(180))), 0.4),
                new GrabSpecimenCommand(),
                new PositionCommand(new Pose2d(100 * specimenNum, 736, new Rotation2d(0)), 0.6).alongWith(new VerticalSlidesCommand(Outtake.SlideState.HIGH_CHAMBER, true)),
                new ScoreSpecimenCommand()
        );
    }
}
