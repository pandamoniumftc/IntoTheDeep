package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ResetVerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;
import org.firstinspires.ftc.teamcode.Util.Spline;

public class PushSampleIntoZoneCommand extends SequentialCommandGroup {
    //-784 - 300 * (num - 1)
    public PushSampleIntoZoneCommand() {
        super(
                new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new PositionCommand(new Pose2d(1239, -508, 0), 0.8),
                new PositionCommand(new Pose2d(1339, -839, Math.toRadians(-180)), 0.6),
                new PositionCommand(new Pose2d(277, -839, Math.toRadians(-180)), 0.8)
        );
    }
}
