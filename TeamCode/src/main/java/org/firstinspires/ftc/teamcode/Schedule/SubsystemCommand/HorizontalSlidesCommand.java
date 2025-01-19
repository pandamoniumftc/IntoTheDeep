package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class HorizontalSlidesCommand extends SequentialCommandGroup {
    public HorizontalSlidesCommand(Intake.SlideState state, boolean wait) {
        super(
                new InstantCommand(() -> PandaRobot.getInstance().intake.updateSlideState(state)),
                new WaitUntilCommand(() -> !wait || PandaRobot.getInstance().horizontalSlideActuator.reached)
        );
    }
}
