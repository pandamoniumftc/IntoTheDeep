package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

public class TransferAndScoreSampleAutoCommand extends SequentialCommandGroup {
    public TransferAndScoreSampleAutoCommand(boolean spline) {
        super(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new TransferSampleCommand(),
                                        new VerticalSlidesCommand(Outtake.SlideState.HIGH_BASKET, true).andThen(new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE))
                                ),
                                new PositionCommand(new Pose2d(189, 433, Math.toRadians(135)), 0.6)
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new TransferSampleCommand(),
                                        new VerticalSlidesCommand(Outtake.SlideState.HIGH_BASKET, true).andThen(new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE))
                                ),
                                new PositionCommand(new Pose2d(189, 433, Math.toRadians(135)), 0.6)
                        ),
                        () -> !spline
                ),
                new WaitCommand(500),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new WaitCommand(250),
                new InstantCommand(() -> PandaRobot.getInstance().intake.retries = 0)
        );
    }
}
