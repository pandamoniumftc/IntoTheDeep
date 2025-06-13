package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.PathLibrary;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabObjectCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.HighChamberCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ResetVerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.RetractIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class CycleSpecimenCommand extends SequentialCommandGroup {
    public CycleSpecimenCommand(int specimenNum) {
        super(
                new PositionCommand(new Pose2d(450, -600, 0.7), 0.8).alongWith(new ResetVerticalSlidesCommand()),
                new ExtendIntakeCommand(),
                new WaitCommand(500),
                new GrabObjectCommand(),
                new SplineCommand(PathLibrary.score).alongWith(
                        new SequentialCommandGroup(
                                new RetractIntakeCommand(),
                                new HighChamberCommand()
                        )
                ),
                new PositionCommand(new Pose2d(840, 250 - 50 * (specimenNum - 1), 0), 0.8),
                new ScoreSpecimenCommand(true),
                new WaitCommand(750)
        );
    }
}
