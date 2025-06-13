package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.HighBasketCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class GoToBasketCommand extends SequentialCommandGroup {
    public GoToBasketCommand() {
        super(
                new PositionCommand(new Pose2d(0, 300, Math.toRadians(90)), 1)
                        .alongWith(
                                new ParallelCommandGroup(
                                        new HighBasketCommand(),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new ExtendIntakeCommand()
                                        )
                                )
                        ),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new WaitCommand(100)
        );
    }
}
