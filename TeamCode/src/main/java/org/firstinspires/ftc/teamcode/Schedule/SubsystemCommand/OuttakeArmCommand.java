package org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

public class OuttakeArmCommand extends SequentialCommandGroup {
    public OuttakeArmCommand(HuaHua robot, Outtake.ArmState state) {
        super(
                new InstantCommand(() -> robot.outtake.updateArmState(state)),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.outtake.moveArm()),
                                new WaitCommand(1500),
                                new InstantCommand(() -> robot.outtake.rotateClaw())
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.outtake.moveArm()),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.outtake.rotateClaw()),
                                new OuttakeClawCommand(robot, Outtake.ClawState.OPENED)
                        ),
                        () -> state == Outtake.ArmState.SCORING
                )
        );
    }
}
