package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class OuttakeClawCommand extends InstantCommand {
    public OuttakeClawCommand(Outtake.ClawState state) {
        super(
                () -> PandaRobot.getInstance().outtake.updateClawState(state)
        );
    }
}
