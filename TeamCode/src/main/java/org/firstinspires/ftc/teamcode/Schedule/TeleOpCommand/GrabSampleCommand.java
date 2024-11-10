package org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand;

import static org.firstinspires.ftc.teamcode.Subsystem.Intake.ClawState.CLOSED;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class GrabSampleCommand extends SequentialCommandGroup {
    public GrabSampleCommand(HuaHua robot) {
        super(
                new IntakeArmCommand(robot, Intake.ArmState.GRABBING),
                new WaitCommand(500),
                new IntakeClawCommand(robot, Intake.ClawState.OPENED),
                new WaitCommand(500),
                new RetractIntakeCommand(robot),
                new WaitCommand(1500),
                new IntakeClawCommand(robot, Intake.ClawState.CLOSED)
        );
    }
}
