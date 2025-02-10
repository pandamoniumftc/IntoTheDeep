package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class ScoreFirstSampleCommand extends SequentialCommandGroup {
    public ScoreFirstSampleCommand() {
        super(
                new ParallelCommandGroup(
                        new PositionCommand(new Pose2d(-239, 509, new Rotation2d(Math.toRadians(-180))), 0.6),
                        new ExtendIntakeCommand(),
                        new VerticalSlidesCommand(Outtake.SlideState.DEFAULT, false),
                        new OuttakeArmCommand(Outtake.ArmState.TRANSFERING)
                ),
                new GrabSampleCommand(),
                new ConditionalCommand(
                        new WaitCommand(0),
                        new SequentialCommandGroup(
                                new ExtendIntakeCommand(),
                                new GrabSampleCommand()
                        ),
                        () -> (PandaRobot.getInstance().controlHub.getLynxModule().getCurrent(CurrentUnit.AMPS) - PandaRobot.getInstance().current) > 0.1
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new TransferSampleCommand(),
                                new VerticalSlidesCommand(Outtake.SlideState.HIGH_BASKET, true)
                        ),
                        new PositionCommand(new Pose2d(-410, 201, new Rotation2d(Math.toRadians(-135))), 0.6)
                ),
                new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE),
                new WaitCommand(500),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new WaitCommand(250)
        );
    }
}
