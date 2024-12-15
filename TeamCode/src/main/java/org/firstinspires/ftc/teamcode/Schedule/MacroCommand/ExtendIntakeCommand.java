package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ExtendIntakeCommand extends SequentialCommandGroup {
    public ExtendIntakeCommand() {
        super(
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                //new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new HorizontalSlidesCommand(240),
                new WaitCommand(500),
                new IntakeArmCommand(Intake.ArmState.INTAKING)
        );
    }
}
