package org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class TransferSampleCommand extends SequentialCommandGroup {
    public TransferSampleCommand(HuaHua robot) {
        super(
                new OuttakeClawCommand(robot, Outtake.ClawState.CLOSED),
                new IntakeClawCommand(robot, Intake.ClawState.CLOSED),
                new HorizontalSlidesCommand(robot, 5),
                new VerticalSlidesCommand(robot, 5),
                new OuttakeArmCommand(robot, Outtake.ArmState.SCORING).alongWith(new HorizontalSlidesCommand(robot, 0))
        );
    }
}
