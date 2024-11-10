package org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class RetractIntakeCommand extends SequentialCommandGroup {
    public RetractIntakeCommand(HuaHua robot) {
        super(
                new IntakeArmCommand(robot, Intake.ArmState.TRANSFERING),
                new InstantCommand(() -> robot.intake.resetClawRotation())
                //new WaitCommand(500),
                //new HorizontalSlidesCommand(robot, 0)
        );
    }
}
