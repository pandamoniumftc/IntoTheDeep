package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class TransferSampleCommand extends SequentialCommandGroup {
    public TransferSampleCommand() {
        super(
                new InstantCommand(() -> PandaRobot.getInstance().intake.retries = 0),
                new RetractIntakeCommand(),
                new OuttakeArmCommand(Outtake.ArmState.CHECKING),
                new WaitCommand(250),
                new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                new WaitCommand(250),
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                new WaitCommand(500),
                new IntakeArmCommand(Intake.ArmState.DEFAULT)
        );
    }
}
