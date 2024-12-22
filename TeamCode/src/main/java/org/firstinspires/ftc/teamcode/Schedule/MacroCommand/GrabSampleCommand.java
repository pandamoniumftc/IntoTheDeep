package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import static com.qualcomm.robotcore.util.Range.scale;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class GrabSampleCommand extends SequentialCommandGroup {
    public GrabSampleCommand() {
        super(
                new InstantCommand(() -> Robot.getInstance().intakeRotateServo.setPosition(scale(Robot.getInstance().sampleAlignmentPipeline.getSampleAngle(), 0, 180, 0.443, 0.3285))),
                new WaitCommand(350),
                new IntakeArmCommand(Intake.ArmState.GRABBING),
                new WaitCommand(200),
                new IntakeClawCommand(Intake.ClawState.OPENED),
                new WaitCommand(350),
                new IntakeArmCommand(Intake.ArmState.INTAKING)
        );
    }
}
