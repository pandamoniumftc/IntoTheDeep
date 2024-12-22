package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class VerticalSlidesCommand extends SequentialCommandGroup {
    public VerticalSlidesCommand(double targetPosition, boolean wait) {
        super(
                new InstantCommand(() -> Robot.getInstance().verticalSlidesActuator.setTargetPosition(targetPosition)),
                new WaitUntilCommand(() -> !wait || Robot.getInstance().verticalSlidesActuator.reached)
        );
    }
}
