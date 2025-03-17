package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@TeleOp(name="main tele op")
public class MainTeleAwp extends LinearOpMode {
    PandaRobot robot = PandaRobot.getInstance();
    GamepadEx Gamepad1, Gamepad2;
    final double MIN_MOTOR_POWER = 0.25;
    long loopStamp = 0L;
    boolean SpecimenScoringMode = false;
    MultipleTelemetry multipleTelemetry;
    double pos = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        Globals.opMode = Globals.RobotOpMode.TELEOP;

        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        robot.initialize(hardwareMap);

        robot.drive.setDriverGamepad(Gamepad1);

        multipleTelemetry = new MultipleTelemetry(telemetry);

        robot.intakeLightChain.setPosition(1);

        robot.odometry.recalibrateIMU();
        robot.odometry.setPosition(Globals.pose);

        // extend and retracts intake arm
        Gamepad2.getGamepadButton(GamepadKeys.Button.B).
                whenPressed(
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new ExtendIntakeCommand(),
                                        new IntakeSampleCommand(),
                                        () -> PandaRobot.getInstance().intake.slideState == Intake.SlideState.TRANSFERRING
                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new OuttakeClawCommand(Outtake.ClawState.CLOSED),
                                                new WaitCommand(500),
                                                new VerticalSlidesCommand(Outtake.SlideState.GRABBED_SPECIMEN, false)
                                        ),
                                        new WaitCommand(0),
                                        () -> PandaRobot.getInstance().outtakeClawSensor.getDistance(DistanceUnit.MM) < 100.0
                                ),
                                () -> !SpecimenScoringMode
                        )
                );

        // retracts slides and transfers object
        Gamepad2.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(
                        new ConditionalCommand(
                                new VerticalSlidesCommand(Outtake.SlideState.HIGH_BASKET, true).andThen(new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE)),
                                new VerticalSlidesCommand(Outtake.SlideState.HIGH_CHAMBER, true).andThen(new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN)),
                                () -> !SpecimenScoringMode
                        )
                );

        // switches between scoring modes
        Gamepad2.getGamepadButton(GamepadKeys.Button.BACK).
                toggleWhenPressed(
                        new InstantCommand(() -> SpecimenScoringMode = true).alongWith(new SequentialCommandGroup(
                                new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN),
                                new WaitCommand(500),
                                new OuttakeClawCommand(Outtake.ClawState.OPENED)
                        )),
                        new InstantCommand(() -> SpecimenScoringMode = false).alongWith(new OuttakeArmCommand(Outtake.ArmState.TRANSFERRING))
                );

        Gamepad2.getGamepadButton(GamepadKeys.Button.X).
                whenPressed(
                        new ConditionalCommand(
                                new VerticalSlidesCommand(Outtake.SlideState.LOW_BASKET, true).andThen(new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE)),
                                new VerticalSlidesCommand(Outtake.SlideState.LOW_CHAMBER, true).andThen(new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN)),
                                () -> !SpecimenScoringMode
                        )
                );

        Gamepad2.getGamepadButton(GamepadKeys.Button.Y).
                whenPressed(
                        new ConditionalCommand(
                                new ScoreSampleCommand(),
                                new ScoreSpecimenCommand(),
                                () -> !SpecimenScoringMode
                        )
                );

        robot.reset(true);

        while (opModeInInit()) {
            robot.update();
            multipleTelemetry.addLine("Initializing...");
            multipleTelemetry.update();
        }

        multipleTelemetry.clear();

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            multipleTelemetry.addData("INTAKE SLIDES POS", robot.horizontalSlideActuator.getPosition());
            telemetry.addData("OUTTAKE SLIDES POS", robot.verticalSlidesActuator.profileOutput + " " + robot.verticalSlidesActuator.getPosition());
            telemetry.addData("DRIVETRAIN POWERS", robot.drive.printMotorPowers());
            telemetry.addData("ROBOT POS", robot.odometry.getPosition().toString());
            //telemetry.addData("LIMELIGHT RESULTS", robot.intake.getLimelightResults());
            //telemetry.addData("INCH", robot.outtakeClawSensor.getDistance(DistanceUnit.INCH));
            //telemetry.addData("STATE", robot.drive.state);
            telemetry.addData("CURRENT", robot.intake.current);
            multipleTelemetry.addData("HZ", 1E9 / (System.nanoTime() - loopStamp));
            loopStamp = System.nanoTime();
            multipleTelemetry.update();
        }

        robot.limelight.shutdown();
    }
}
