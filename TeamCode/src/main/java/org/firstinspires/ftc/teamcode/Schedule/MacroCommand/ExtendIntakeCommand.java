package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ExtendIntakeCommand extends ConditionalCommand {
    public ExtendIntakeCommand() {
        super(
                new SequentialCommandGroup(
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new HorizontalSlidesCommand(175),
                        new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new IntakeArmCommand(Intake.ArmState.INTAKING)
                ),
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                () -> Robot.getInstance().intake.armState == Intake.ArmState.TRANSFERING || Robot.getInstance().intake.armState == Intake.ArmState.DEFAULT
        );
    }
}
