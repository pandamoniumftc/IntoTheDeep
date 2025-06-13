package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ScoreSampleCommand extends SequentialCommandGroup {
    public ScoreSampleCommand() {
        super(
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new WaitCommand(100),
                new ResetVerticalSlidesCommand()
        );
    }
}
