package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static com.qualcomm.robotcore.util.Range.clip;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.Robots.Globals;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.GrabSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose;

@TeleOp (name="main tele op")
public class MainTeleAwp extends AbstractTeleOp {
    HuaHua robot;
    final double MIN_MOTOR_POWER = 0.4;
    long loopStamp = 0L;
    boolean SpecimenScoringMode = false;
    @Override
    public AbstractRobot instantiateRobot() {
        robot = new HuaHua(this);

        return robot;
    }

    @Override
    public void onInit() {
        robot.imu.resetYaw();
        robot.enableTelemetry(true);

        // reset angle
        robot.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> robot.resetAngle());

        // extend intake arm
        robot.gamepad2.getGamepadButton(GamepadKeys.Button.B).
                whenPressed(
                        new ExtendIntakeCommand(robot).whenFinished(() -> robot.gamepad2.gamepad.rumble(150))
                );

        // grabs object
        robot.gamepad2.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(
                        new GrabSampleCommand(robot).whenFinished(() -> robot.gamepad2.gamepad.rumble(150))
                );

        // extends and retract outtake arm when specimen mode
        robot.gamepad2.getGamepadButton(GamepadKeys.Button.X).
                toggleWhenPressed(
                        new ConditionalCommand(
                                new OuttakeArmCommand(robot, Outtake.ArmState.SCORING),
                                new WaitCommand(0),
                                () -> SpecimenScoringMode
                        ),
                        new ConditionalCommand(
                                new OuttakeArmCommand(robot, Outtake.ArmState.TRANSFERING),
                                new WaitCommand(0),
                                () -> SpecimenScoringMode
                        )
                );

        // switches between scoring modes
        robot.gamepad2.getGamepadButton(GamepadKeys.Button.START).
                toggleWhenPressed(
                        new InstantCommand(() -> SpecimenScoringMode = true),
                        new InstantCommand(() -> SpecimenScoringMode = false)
                );

        CommandScheduler.getInstance().schedule(new IntakeArmCommand(robot, robot.intake.armState));
        CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.resetClawRotation()));
        CommandScheduler.getInstance().schedule(new IntakeClawCommand(robot, robot.intake.clawState));
        CommandScheduler.getInstance().schedule(new OuttakeArmCommand(robot, robot.outtake.armState));
        CommandScheduler.getInstance().schedule(new OuttakeClawCommand(robot, robot.outtake.clawState));
    }

    @Override
    public void onDriverUpdate() {
        CommandScheduler.getInstance().run();
        robot.expansionHub.updateBulkData();
        robot.expansionHub.clearBulkData();
        robot.controlHub.updateVoltage();
        robot.expansionHub.updateVoltage();

        // ----------------- MECANUM DRIVE -----------------
        double scale = (1.0 - MIN_MOTOR_POWER) * (1.0 - robot.gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) + MIN_MOTOR_POWER;

        robot.drive.moveRobot(new Pose(
                        robot.gamepad1.getLeftX() * scale, robot.gamepad1.getLeftY() * scale, robot.gamepad1.getRightX() * scale),
                robot.getAngle(AngleUnit.RADIANS)
        );

        robot.drive.setDrivePower();
        robot.telemetry.addData("angle", robot.getAngle(AngleUnit.DEGREES));

        // ------------------- INTAKE --------------------
        if (robot.intake.armState == Intake.ArmState.INTAKING) {
            robot.intake.rotateClaw(robot.gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), robot.gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        }

        if (robot.intake.armState == Intake.ArmState.TRANSFERING && robot.intake.clawState == Intake.ClawState.OPENED && robot.intake.slideM.reachedPosition(0.08) && robot.intake.slideM.getPosition() < 0.5) {
            CommandScheduler.getInstance().schedule(new TransferSampleCommand(robot));
        }

        robot.intake.slideM.setVoltage(robot.controlHub.getVoltage());
        robot.intake.slideM.update();

        // ------------------- OUTTAKE ---------------------

        if (robot.outtake.getSlidesPosition() > 9 && robot.outtake.armState == Outtake.ArmState.TRANSFERING && !SpecimenScoringMode) {
            CommandScheduler.getInstance().schedule(new OuttakeArmCommand(robot, Outtake.ArmState.SCORING));
        }
        else if (robot.outtake.getSlidesPosition() < 9 && robot.outtake.armState == Outtake.ArmState.SCORING && !SpecimenScoringMode) {
            CommandScheduler.getInstance().schedule(new OuttakeArmCommand(robot, Outtake.ArmState.TRANSFERING));
        }

        if (robot.outtake.armState == Outtake.ArmState.SCORING) {
            robot.outtake.updateClawState(robot.gamepad2.getGamepadButton(GamepadKeys.Button.Y).get() ? Outtake.ClawState.OPENED : Outtake.ClawState.CLOSED);
        }

        robot.outtake.moveSlides(robot.gamepad2.getLeftY());

        robot.telemetry.addData("HZ", 1E9 / (System.nanoTime() - loopStamp));
        loopStamp = System.nanoTime();

        if (Globals.telemetryEnable) {
            telemetry.update();
        }
    }

    @Override
    public void onStop() {

    }

}