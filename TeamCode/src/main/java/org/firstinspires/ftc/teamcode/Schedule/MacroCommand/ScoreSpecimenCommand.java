package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ScoreSpecimenCommand extends ConditionalCommand {
    public ScoreSpecimenCommand(boolean auto) {
        super(
                new SequentialCommandGroup(
                        new VerticalSlidesCommand(Outtake.SlideState.SCORED_CHAMBER, true),
                        new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new ResetVerticalSlidesCommand()
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                                new ResetVerticalSlidesCommand()
                        ),
                        new VerticalSlidesCommand(Outtake.SlideState.SCORED_CHAMBER, true),
                        () -> PandaRobot.getInstance().outtake.slideState == Outtake.SlideState.SCORED_CHAMBER
                ),

                () -> auto
        );
    }
}
