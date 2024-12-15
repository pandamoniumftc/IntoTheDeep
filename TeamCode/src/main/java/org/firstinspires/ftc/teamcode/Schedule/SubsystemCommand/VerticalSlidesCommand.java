package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class VerticalSlidesCommand extends ParallelCommandGroup {
    public VerticalSlidesCommand(double targetPosition) {
        super(
                new InstantCommand(() -> Robot.getInstance().verticalSlidesActuator.setTargetPosition(targetPosition))
        );
    }
}
