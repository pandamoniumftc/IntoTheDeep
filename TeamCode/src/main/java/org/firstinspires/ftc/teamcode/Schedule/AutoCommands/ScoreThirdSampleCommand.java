package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ResetVerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class ScoreThirdSampleCommand extends SequentialCommandGroup {
    public ScoreThirdSampleCommand() {
        super(

            new ParallelCommandGroup(
                    new PositionCommand(new Pose2d(443, 379, -2.53), 0.6),
                    new ExtendIntakeCommand(),
                    new ResetVerticalSlidesCommand(false)
            ),
                new IntakeSampleCommand()
        );
    }
}
