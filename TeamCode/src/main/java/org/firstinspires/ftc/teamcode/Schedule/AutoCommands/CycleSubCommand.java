package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Hardware.PathLibrary;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabObjectCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.HighBasketCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ResetVerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.RetractIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class CycleSubCommand extends SequentialCommandGroup {
    public CycleSubCommand() {
        super(
                new SplineCommand(PathLibrary.basketToSub).alongWith(
                        new SequentialCommandGroup(
                                new ResetVerticalSlidesCommand(),
                                new ExtendIntakeCommand()
                        )
                ),
                new WaitCommand(500),
                new ConditionalCommand(
                        new InstantCommand(),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> Globals.targetingColorPiece = true),
                                new WaitCommand(500)
                        ),
                        () -> PandaRobot.getInstance().intake.objectDetected()
                ),
                new GrabObjectCommand(),
                new SplineCommand(PathLibrary.subToBasket).alongWith(
                        new SequentialCommandGroup(
                                new RetractIntakeCommand(),
                                new HighBasketCommand()
                        )
                ),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new WaitCommand(100),
                new ResetVerticalSlidesCommand()
        );
    }
}
