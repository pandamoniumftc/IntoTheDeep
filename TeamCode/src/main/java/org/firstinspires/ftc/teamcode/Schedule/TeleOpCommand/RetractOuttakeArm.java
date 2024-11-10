package org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class RetractOuttakeArm extends SequentialCommandGroup {
    public RetractOuttakeArm(HuaHua robot) {
        super(
                new OuttakeClawCommand(robot, Outtake.ClawState.CLOSED),
                new OuttakeArmCommand(robot, Outtake.ArmState.TRANSFERING),
                new VerticalSlidesCommand(robot, 0)
        );
    }
}
