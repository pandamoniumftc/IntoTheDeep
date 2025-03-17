package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.opencv.core.Point;

public class ParkFirstLevelAscentCommand extends SequentialCommandGroup {
    public ParkFirstLevelAscentCommand() {
        super(
                new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN)
        );
    }
}
