package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ScoreFirstSampleCommand extends SequentialCommandGroup {
    public ScoreFirstSampleCommand() {
        super(
                new ParallelCommandGroup(
                        new PositionCommand(new Pose2d(new Translation2d(-200, 300), new Rotation2d(-PI))),
                        new VerticalSlidesCommand(0, false),
                        new OuttakeArmCommand(Outtake.ArmState.TRANSFERING)
                ),
                new ExtendIntakeCommand(),
                new WaitCommand(1000),
                new GrabSampleCommand(),
                new TransferSampleCommand(),
                new PositionCommand(new Pose2d(new Translation2d(-375.0, 125), new Rotation2d(-3 * PI / 4))).alongWith(new VerticalSlidesCommand(3750, false)),
                new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE),
                new WaitCommand(500),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new WaitCommand(250)
        );
    }
}
