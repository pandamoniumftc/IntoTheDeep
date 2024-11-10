package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class OuttakeArmCommand extends InstantCommand {
    public OuttakeArmCommand(HuaHua robot, Outtake.ArmState state) {
        super(
                () -> robot.outtake.updateArmState(state)
        );
    }
}
