package org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class TransferSampleCommand extends SequentialCommandGroup {
    public TransferSampleCommand(HuaHua robot) {
        super(
                new WaitCommand(1000),
                new OuttakeClawCommand(robot, Outtake.ClawState.CLOSED),
                new WaitCommand(100),
                new IntakeClawCommand(robot, Intake.ClawState.CLOSED),
                new WaitCommand(500),
                new IntakeArmCommand(robot, Intake.ArmState.DEFAULT)
        );
    }
}
