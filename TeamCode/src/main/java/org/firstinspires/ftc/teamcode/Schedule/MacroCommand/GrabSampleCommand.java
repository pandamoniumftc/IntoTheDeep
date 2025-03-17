package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.AdjustPositionToSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class GrabSampleCommand extends SequentialCommandGroup {
    public GrabSampleCommand() {
        super(
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                new IntakeArmCommand(Intake.ArmState.SCANNING),
                new WaitCommand(250),
                new AdjustPositionToSampleCommand(),
                new IntakeArmCommand(Intake.ArmState.ROTATE),
                new WaitCommand(750),
                new IntakeArmCommand(Intake.ArmState.GRABBING),
                new WaitCommand(500),
                new InstantCommand(() -> PandaRobot.getInstance().current = PandaRobot.getInstance().controlHub.getLynxModule().getCurrent(CurrentUnit.AMPS)),
                new IntakeClawCommand(Intake.ClawState.OPENED),
                new WaitCommand(500),
                new InstantCommand(() -> PandaRobot.getInstance().intake.current = PandaRobot.getInstance().controlHub.getLynxModule().getCurrent(CurrentUnit.AMPS) - PandaRobot.getInstance().current)
        );
    }
}
