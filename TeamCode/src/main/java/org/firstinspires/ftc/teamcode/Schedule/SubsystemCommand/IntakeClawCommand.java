package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class IntakeClawCommand extends InstantCommand {
    public IntakeClawCommand(HuaHua robot, Intake.ClawState state) {
        super(
                () -> robot.intake.updateClawState(state)
        );
    }
}
