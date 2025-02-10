package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class RetractIntakeCommand extends SequentialCommandGroup {
    public RetractIntakeCommand() {
        super(
                new IntakeArmCommand(Intake.ArmState.DEFAULT),
                new WaitCommand(500),
                new IntakeArmCommand(Intake.ArmState.TRANSFERRING),
                new WaitCommand(500),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new HorizontalSlidesCommand(Intake.SlideState.TRANSFERRING, true)
        );
    }
}
