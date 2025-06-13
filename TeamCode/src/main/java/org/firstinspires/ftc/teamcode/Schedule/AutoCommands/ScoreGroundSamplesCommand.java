package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabObjectCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.HighBasketCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ResetVerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.RetractIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;
public class ScoreGroundSamplesCommand extends SequentialCommandGroup {
    public static Pose2d first = new Pose2d(467, 325, 2.96);
    public static Pose2d second = new Pose2d(463, 424, -3.14);
    public static Pose2d third = new Pose2d(700, 237, -2.16);

    public ScoreGroundSamplesCommand(int num) {
        super(createCommandGroup(num));
    }

    private static SequentialCommandGroup createCommandGroup(int num) {
        Pose2d pos;
        switch (num) {
            case 1:
                pos = first;
                break;
            case 2:
                pos = second;
                break;
            case 3:
                pos = third;
                break;
            default:
                throw new IllegalArgumentException("Invalid sample number: " + num);
        }

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PositionCommand(pos, 1),
                        new ResetVerticalSlidesCommand()
                ),
                new WaitCommand(500),
                new GrabObjectCommand(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new RetractIntakeCommand(),
                                new ParallelCommandGroup(
                                        new HighBasketCommand(),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new ExtendIntakeCommand()
                                        )
                                )
                        ),
                        new PositionCommand(new Pose2d(200, 450, 2.34), 1)
                ),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new WaitCommand(100)
        );
    }
}
