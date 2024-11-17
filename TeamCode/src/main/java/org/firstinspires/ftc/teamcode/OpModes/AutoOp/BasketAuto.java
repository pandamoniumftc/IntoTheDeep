package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import static org.firstinspires.ftc.teamcode.Schedule.DriveCommand.DrivePowerCommand.Direction.FORWARD;
import static java.util.concurrent.TimeUnit.SECONDS;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.DrivePowerCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.GrabSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.RetractIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
@Autonomous (name = "basket auto")
public class BasketAuto extends AbstractAutonomous {
    HuaHua robot;
    @Override
    public AbstractRobot instantiateRobot() {
        robot = new HuaHua(this);
        return robot;
    }

    @Override
    public void onInit() {
        robot.enableTelemetry(true);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new IntakeArmCommand(robot, robot.intake.armState),
                        new InstantCommand(() -> robot.intake.resetClawRotation()),
                        new OuttakeArmCommand(robot, robot.outtake.armState),
                        new OuttakeClawCommand(robot, robot.outtake.clawState),
                        new IntakeClawCommand(robot, Intake.ClawState.CLOSED),
                        new WaitCommand(3000),
                        new IntakeClawCommand(robot, Intake.ClawState.OPENED)
                )
        );
    }

    @Override
    public Command autonomous() {
        return new SequentialCommandGroup(
                new HorizontalSlidesCommand(robot, 10),
                new WaitCommand(500),
                new IntakeArmCommand(robot, Intake.ArmState.INTAKING),
                new WaitUntilCommand(() -> robot.intake.slideM.reachedPosition(0.1)),
                new IntakeClawCommand(robot, Intake.ClawState.CLOSED),
                new WaitCommand(1000),
                new RetractIntakeCommand(robot),
                new WaitUntilCommand(() -> autonomousTime.time(SECONDS) > 20),
                new DrivePowerCommand(robot, FORWARD, 4, 0.5)
        );
    }
}
