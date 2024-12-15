package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class TransferSampleCommand extends SequentialCommandGroup {
    public TransferSampleCommand() {
        super(
                new RetractIntakeCommand(),
                new WaitUntilCommand(() -> Robot.getInstance().horizontalSlideActuator.reached && Robot.getInstance().horizontalSlideActuator.getPosition() < 50),
                //new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                new WaitCommand(250),
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                new WaitCommand(250),
                new IntakeArmCommand(Intake.ArmState.DEFAULT)
        );
    }
}
