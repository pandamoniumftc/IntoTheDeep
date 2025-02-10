package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class GrabSpecimenCommand extends ConditionalCommand {
    public GrabSpecimenCommand() {
        super(
                new SequentialCommandGroup(
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                        new WaitCommand(500),
                        new VerticalSlidesCommand(Outtake.SlideState.GRABBED_SPECIMEN, true)
                ),
                new WaitCommand(0),
                () -> true
        );
    }
}
