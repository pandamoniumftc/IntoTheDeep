package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;

public class VerticalSlidesCommand extends SequentialCommandGroup {
    public VerticalSlidesCommand(double targetPosition, boolean wait) {
        super(
                new InstantCommand(() -> PandaRobot.getInstance().verticalSlidesActuator.setTargetPosition(targetPosition)),
                new WaitUntilCommand(() -> !wait || PandaRobot.getInstance().verticalSlidesActuator.reached)
        );
    }
}
