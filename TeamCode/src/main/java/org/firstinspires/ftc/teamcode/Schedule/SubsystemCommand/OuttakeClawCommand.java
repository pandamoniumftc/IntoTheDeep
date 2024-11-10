package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class OuttakeClawCommand extends InstantCommand {
    public OuttakeClawCommand(HuaHua robot, Outtake.ClawState state) {
        super(
                () -> robot.outtake.updateClawState(state)
        );
    }
}
