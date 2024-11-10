package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;

public class VerticalSlidesCommand extends ParallelCommandGroup {
    public VerticalSlidesCommand(HuaHua robot, double targetPosition) {
        super(
                new InstantCommand(() -> robot.outtake.leftMotor.setPosition(targetPosition)),
                new InstantCommand(() -> robot.outtake.rightMotor.setPosition(targetPosition))
        );
    }
}
