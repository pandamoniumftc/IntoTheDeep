package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.AdjustPositionToSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class GrabObjectCommand extends ConditionalCommand {
    public GrabObjectCommand() {
        super(
                new ConditionalCommand(
                        new DropObjectCommand(),
                        new SequentialCommandGroup(
                                new IntakeArmCommand(Intake.ArmState.RAISED),
                                new AdjustPositionToSampleCommand(),
                                new WaitCommand(250),
                                new IntakeArmCommand(Intake.ArmState.GRABBING),
                                new WaitCommand(400),
                                new IntakeClawCommand(Intake.ClawState.CLOSED),
                                new WaitCommand(250),
                                new IntakeArmCommand(Intake.ArmState.RAISED)
                        ),
                        () -> PandaRobot.getInstance().intake.armState == Intake.ArmState.RAISED
                ),
                new InstantCommand(),
                () ->  PandaRobot.getInstance().intake.slideState == Intake.SlideState.GRABBING_SAMPLE
        );
    }
}
