package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class OuttakeArmCommand extends SequentialCommandGroup {
    public OuttakeArmCommand(Outtake.ArmState state) {
        super(
                new InstantCommand(() -> Robot.getInstance().outtake.updateArmState(state))
        );
    }
}
