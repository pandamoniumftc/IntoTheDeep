package org.firstinspires.ftc.teamcode.Schedule.AutoCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.opencv.core.Point;

public class GoToBasketCommand extends SequentialCommandGroup {
    public GoToBasketCommand() {
        super(
                new ParallelCommandGroup(
                        new SplineCommand(new Pose2d(), 0, 0, 0.4),
                        new VerticalSlidesCommand(Outtake.SlideState.DEFAULT, false),
                        new OuttakeArmCommand(Outtake.ArmState.TRANSFERING)
                )
        );
    }
}
