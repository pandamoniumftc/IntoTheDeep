package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class ScoreSpecimenPreloadRightSideCommand extends SequentialCommandGroup {
    public ScoreSpecimenPreloadRightSideCommand() {
        super(
                new ParallelCommandGroup(
                        new PositionCommand(new Pose2d(654, 0, 0), 0.7),
                        new VerticalSlidesCommand(Outtake.SlideState.HIGH_CHAMBER, true)
                ),
                new PositionCommand(new Pose2d(736, 0, 0), 0.7),
                new ScoreSpecimenCommand()
        );
    }
}
