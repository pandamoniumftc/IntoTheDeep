package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class HighChamberCommand extends SequentialCommandGroup {
    public HighChamberCommand() {
        super(
                new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN),
                new VerticalSlidesCommand(Outtake.SlideState.HIGH_CHAMBER, true)
        );
    }
}
