package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class RetractIntakeCommand extends SequentialCommandGroup {
    public RetractIntakeCommand() {
        super(
                new IntakeArmCommand(Intake.ArmState.TRANSFERING),
                new WaitCommand(250),
                new HorizontalSlidesCommand(0)
        );
    }
}
