package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ExtendIntakeCommand extends SequentialCommandGroup {
    public ExtendIntakeCommand() {
        super(
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                new OuttakeArmCommand(Outtake.ArmState.CHECKING),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new HorizontalSlidesCommand(Intake.SlideState.GRABBING_SAMPLE, true),
                new IntakeArmCommand(Intake.ArmState.SCANNING)
        );
    }
}
