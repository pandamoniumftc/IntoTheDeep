package org.firstinspires.ftc.teamcode.Schedule.MacroCommand;

import static com.qualcomm.robotcore.util.Range.scale;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.AdjustPositionToSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

public class GrabSampleCommand extends SequentialCommandGroup {
    public GrabSampleCommand() {
        super(
                new AdjustPositionToSampleCommand(),
                new InstantCommand(() -> PandaRobot.getInstance().intakeRotateClawServo.setPosition(scale(PandaRobot.getInstance().sampleAlignmentPipeline.getSampleAngle(), 0, 180, 0.445, 0.326))),
                new HorizontalSlidesCommand(Intake.SlideState.GRABBING_SAMPLE, true),
                new IntakeArmCommand(Intake.ArmState.GRABBING),
                new WaitCommand(1000),
                new IntakeClawCommand(Intake.ClawState.OPENED),
                new WaitCommand(250),
                new IntakeArmCommand(Intake.ArmState.RETRACT)
        );
    }
}
