package org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ExtendOuttakeArm extends SequentialCommandGroup {
    public ExtendOuttakeArm(HuaHua robot) {
        super(
                new VerticalSlidesCommand(robot, 5),
                new WaitCommand(500),
                new OuttakeArmCommand(robot, Outtake.ArmState.SCORING)
        );
    }
}
