package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class RetractIntakeCommand extends SequentialCommandGroup {
    public RetractIntakeCommand() {
        super(
                new IntakeArmCommand(Intake.ArmState.TRANSFERRING),
                new OuttakeArmCommand(Outtake.ArmState.TRANSFERRING),
                new HorizontalSlidesCommand(Intake.SlideState.TRANSFERRING, true),
                new WaitCommand(250),
                new TransferObjectCommand()
        );
    }
}
