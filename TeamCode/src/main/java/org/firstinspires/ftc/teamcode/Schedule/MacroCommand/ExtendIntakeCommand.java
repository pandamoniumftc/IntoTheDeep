package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ExtendIntakeCommand extends ConditionalCommand {
    public ExtendIntakeCommand() {
        super(
                new ParallelCommandGroup(
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new HorizontalSlidesCommand(Intake.SlideState.SCANNING_SAMPLE, true),
                        new IntakeArmCommand(Intake.ArmState.DEFAULT)
                ),
                new ParallelCommandGroup(
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new IntakeArmCommand(Intake.ArmState.DEFAULT)
                ),
                () -> PandaRobot.getInstance().intake.slideState != Intake.SlideState.GRABBING_SAMPLE || PandaRobot.getInstance().intake.slideState != Intake.SlideState.SCANNING_SAMPLE
        );
    }
}
