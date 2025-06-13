package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class HighBasketCommand extends SequentialCommandGroup {
    public HighBasketCommand() {
        super(
                new VerticalSlidesCommand(Outtake.SlideState.HIGH_BASKET, true).alongWith(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> PandaRobot.getInstance().verticalSlidesActuator.getPosition() > 700),
                                new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE)
                        )
                )
        );
    }
}
