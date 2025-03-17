package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ResetVerticalSlidesCommand extends SequentialCommandGroup {
    public ResetVerticalSlidesCommand(boolean SpecimenScoringMode) {
        super(
                new VerticalSlidesCommand(Outtake.SlideState.DEFAULT, false),
                new ConditionalCommand(
                        new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN),
                        new OuttakeArmCommand(Outtake.ArmState.CHECKING),
                        () -> SpecimenScoringMode
                )
        );
    }
}
