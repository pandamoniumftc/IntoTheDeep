package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ResetVerticalSlidesCommand extends SequentialCommandGroup {
    public ResetVerticalSlidesCommand() {
        super(
                new OuttakeArmCommand(Outtake.ArmState.TRANSFERRING),
                new WaitCommand(250),
                new VerticalSlidesCommand(Outtake.SlideState.DEFAULT, false)
        );
    }
}
