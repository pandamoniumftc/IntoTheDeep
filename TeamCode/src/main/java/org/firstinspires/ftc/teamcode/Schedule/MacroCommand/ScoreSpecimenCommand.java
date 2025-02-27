package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ScoreSpecimenCommand extends SequentialCommandGroup {
    public ScoreSpecimenCommand() {
        super(
                new VerticalSlidesCommand(Outtake.SlideState.LOWERED_FROM_HIGH, true),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new WaitCommand(500),
                new VerticalSlidesCommand(Outtake.SlideState.DEFAULT, false)
        );
    }
}
