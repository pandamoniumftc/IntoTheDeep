package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class ParkObservationZone extends SequentialCommandGroup {
    public ParkObservationZone() {
        super(
                new PositionCommand(new Pose2d(169, -1070, Math.toRadians(0)), 0.8)
        );
    }
}
