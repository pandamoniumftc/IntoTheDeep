package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.TransferAndScoreSampleAutoCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class IntakeSampleCommand extends SequentialCommandGroup {
    public IntakeSampleCommand() {
        super(
                new GrabSampleCommand(),
                new ConditionalCommand(
                        new ConditionalCommand(
                                new TransferAndScoreSampleAutoCommand(false),
                                new TransferSampleCommand(),
                                () -> Globals.opMode == Globals.RobotOpMode.AUTO
                        ),
                        new IntakeArmCommand(Intake.ArmState.SCANNING),
                        () -> (PandaRobot.getInstance().intake.current) > 0.10
                )
        );
    }
}
