package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class TransferSampleCommand extends SequentialCommandGroup {
    public TransferSampleCommand() {
        super(
                new RetractIntakeCommand(),
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                new WaitCommand(100),
                new HorizontalSlidesCommand(Intake.SlideState.DEFAULT, true),
                new ConditionalCommand(
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                        new WaitCommand(0),
                        () -> PandaRobot.getInstance().outtakeClawSensor.getDistance(DistanceUnit.MM) < 50.0
                )
        );
    }
}
