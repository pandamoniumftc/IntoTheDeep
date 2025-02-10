package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;

public class ParkObservationZone extends SequentialCommandGroup {
    public ParkObservationZone() {
        super(
                new PositionCommand(new Pose2d(1070, 169, new Rotation2d(Math.toRadians(0))), 0.7)
        );
    }
}
