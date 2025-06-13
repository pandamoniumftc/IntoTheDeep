package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Hardware.PathLibrary;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ResetVerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class PushSampleIntoZoneCommand extends SequentialCommandGroup {
    //-784 - 300 * (num - 1)
    public PushSampleIntoZoneCommand() {
        super(
                new SplineCommand(PathLibrary.push1).alongWith(new ResetVerticalSlidesCommand()),
                /*new PositionCommand(new Pose2d(550, -650, 2.23), 1.0).alongWith(
                        new SequentialCommandGroup(
                                new ResetVerticalSlidesCommand()
                        )
                ),
                new ExtendIntakeCommand(),
                new WaitCommand(1000),
                new GrabSampleCommand(),
                new PositionCommand(new Pose2d(450, -650, 0.7), 1.0),
                new DropObjectCommand(),
                new PositionCommand(new Pose2d(550, -950, 2.23), 1.0),
                new WaitCommand(1000),
                new GrabSampleCommand(),
                new PositionCommand(new Pose2d(450, -950, 0.7), 1.0),
                new DropObjectCommand()*/
                new PositionCommand(new Pose2d(300, -1100, 0), 0.8),
                new SplineCommand(PathLibrary.push2),
                new PositionCommand(new Pose2d(300, -1300, 0), 0.8)
                //new SplineCommand(PathLibrary.push3),
                //new PositionCommand(new Pose2d(300, -1425, 0), 1)
                //new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                //new WaitCommand(500),
                //new OuttakeArmCommand(Outtake.ArmState.TRANSFERRED),
                //new SplineCommand(PathLibrary.score1)
        );
    }
}
