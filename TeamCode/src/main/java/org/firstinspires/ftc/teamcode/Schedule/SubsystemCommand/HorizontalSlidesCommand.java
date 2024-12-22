package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class HorizontalSlidesCommand extends InstantCommand {
    public HorizontalSlidesCommand(double pos) {
        super(
                () -> Robot.getInstance().horizontalSlideActuator.setTargetPosition(pos)
        );
    }
}
