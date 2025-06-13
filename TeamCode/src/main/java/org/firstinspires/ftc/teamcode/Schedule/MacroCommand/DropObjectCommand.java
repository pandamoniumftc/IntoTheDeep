package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class DropObjectCommand extends SequentialCommandGroup {
    public DropObjectCommand() {
        super(
                new HorizontalSlidesCommand(Intake.SlideState.GRABBING_SAMPLE, true),
                new IntakeClawCommand(Intake.ClawState.OPENED),
                new WaitCommand(500),
                new IntakeArmCommand(Intake.ArmState.SCANNING)

        );
    }
}
