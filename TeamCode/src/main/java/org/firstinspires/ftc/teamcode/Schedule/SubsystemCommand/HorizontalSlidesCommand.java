package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

public class HorizontalSlidesCommand extends InstantCommand {
    public HorizontalSlidesCommand(double millimeter) {
        super(
                () -> Robot.getInstance().horizontalSlideActuator.setTargetPosition(millimeter)
        );
    }
}
