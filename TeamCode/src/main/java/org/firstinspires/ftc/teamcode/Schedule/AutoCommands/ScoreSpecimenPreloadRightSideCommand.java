package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ScoreSpecimenPreloadRightSideCommand extends SequentialCommandGroup {
    public ScoreSpecimenPreloadRightSideCommand() {
        super(
                new ParallelCommandGroup(
                        new PositionCommand(new Pose2d()),
                        new VerticalSlidesCommand(Outtake.SlideState.HIGH_CHAMBER, true),
                        new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE)
                ),
                new ScoreSpecimenCommand()
        );
    }
}
