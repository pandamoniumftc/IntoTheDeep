package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class IntakeArmCommand extends SequentialCommandGroup {
    public IntakeArmCommand(Intake.ArmState state) {
        super(
                new InstantCommand(() -> PandaRobot.getInstance().intake.updateArmState(state))
        );
    }
}
