package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class IntakeClawCommand extends InstantCommand {
    public IntakeClawCommand(Intake.ClawState state) {
        super(
                () -> Robot.getInstance().intake.updateClawState(state)
        );
    }
}
