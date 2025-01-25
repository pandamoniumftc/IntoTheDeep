package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class VerticalSlidesCommand extends SequentialCommandGroup {
    public VerticalSlidesCommand(Outtake.SlideState state, boolean wait) {
        super(
                new InstantCommand(() -> PandaRobot.getInstance().outtake.updateSlideState(state)),
                new WaitUntilCommand(() -> !wait || PandaRobot.getInstance().verticalSlidesActuator.isFinished())
        );
    }
}
