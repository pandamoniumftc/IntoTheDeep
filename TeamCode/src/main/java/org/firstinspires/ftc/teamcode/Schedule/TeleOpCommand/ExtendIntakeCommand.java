package org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ExtendIntakeCommand extends SequentialCommandGroup {
    public ExtendIntakeCommand(HuaHua robot) {
        super(
                new HorizontalSlidesCommand(robot, 10),
                new OuttakeClawCommand(robot, Outtake.ClawState.OPENED),
                new WaitCommand(500),
                new IntakeArmCommand(robot, Intake.ArmState.INTAKING),
                new WaitCommand(500),
                new IntakeClawCommand(robot, Intake.ClawState.CLOSED)
        );
    }
}
