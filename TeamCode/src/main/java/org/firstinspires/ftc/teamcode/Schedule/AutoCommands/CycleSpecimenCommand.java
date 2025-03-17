package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class CycleSpecimenCommand extends SequentialCommandGroup {
    public CycleSpecimenCommand(double specimenNum) {
        super(
                new PositionCommand(new Pose2d(169, -650, Math.toRadians(178)), 0.8),
                new WaitCommand(100),
                new PositionCommand(new Pose2d(100, -650, Math.toRadians(178)), 0.4),
                new SequentialCommandGroup(
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                        new WaitCommand(250),
                        new VerticalSlidesCommand(Outtake.SlideState.HIGH_CHAMBER, false),
                        new WaitCommand(250)
                ),
                new PositionCommand(new Pose2d(750, 300 - 50 * (specimenNum - 1), Math.toRadians(0)), 0.8),
                new ScoreSpecimenCommand()
        );
    }
}
