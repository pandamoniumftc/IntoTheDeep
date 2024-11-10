package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class IntakeArmCommand extends SequentialCommandGroup {
    public IntakeArmCommand(HuaHua robot, Intake.ArmState state) {
        super(
                new InstantCommand(() -> robot.intake.updateArmState(state)),
                new InstantCommand(() -> robot.intake.moveElbow()),
                new InstantCommand(() -> robot.intake.moveWrist())
        );
    }
}
