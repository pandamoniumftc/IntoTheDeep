package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;

public class HorizontalSlidesCommand extends InstantCommand {
    public HorizontalSlidesCommand(HuaHua robot, double targetPosition) {
        super(
                () -> robot.intake.hSlideM.setPosition(targetPosition)
        );
    }
}
