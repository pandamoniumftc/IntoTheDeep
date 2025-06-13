package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.HighChamberCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class ScoreSpecimenPreloadRightSideCommand extends SequentialCommandGroup {
    public ScoreSpecimenPreloadRightSideCommand() {
        super(
                new ParallelCommandGroup(
                        new PositionCommand(new Pose2d(840, 0, 0), 0.8).alongWith(new HighChamberCommand()) //887
                ),
                new ScoreSpecimenCommand(true),
                new WaitCommand(750),
                new OuttakeClawCommand(Outtake.ClawState.OPENED)
        );
    }
}
