package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static com.qualcomm.robotcore.util.Range.scale;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.AdjustPositionToSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
@TeleOp (name="main tele op reset")
public class MainTeleAwpReset extends LinearOpMode {
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
        multipleTelemetry = new MultipleTelemetry(telemetry);

        robot.odometry.resetPosAndIMU();

        robot.read();
        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        robot.intakeLightChain.setPosition(0.5);

        // extend and retracts intake arm
        Gamepad2.getGamepadButton(GamepadKeys.Button.B).
                whenPressed(
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new ExtendIntakeCommand(),
                                        new SequentialCommandGroup(
                                                new AdjustPositionToSampleCommand(),
                                                new InstantCommand(() -> PandaRobot.getInstance().intakeRotateClawServo.setPosition(scale(PandaRobot.getInstance().sampleAlignmentPipeline.getSampleAngle(), 0, 180, 0.445, 0.326))),
                                                new HorizontalSlidesCommand(Intake.SlideState.GRABBING_SAMPLE, true),
                                                new WaitCommand(500),
                                                new IntakeArmCommand(Intake.ArmState.GRABBING),
                                                new WaitCommand(250),
                                                new InstantCommand(() -> PandaRobot.getInstance().current = PandaRobot.getInstance().controlHub.getLynxModule().getCurrent(CurrentUnit.AMPS)),
                                                new IntakeClawCommand(Intake.ClawState.OPENED),
                                                new WaitCommand(500),
                                                new ConditionalCommand(
                                                        new TransferSampleCommand(),
                                                        new ExtendIntakeCommand(),
                                                        () -> (PandaRobot.getInstance().controlHub.getLynxModule().getCurrent(CurrentUnit.AMPS) - PandaRobot.getInstance().current) > 0.1
                                                )
                                        ),
                                        () -> PandaRobot.getInstance().intake.slideState == Intake.SlideState.DEFAULT || PandaRobot.getInstance().intake.slideState == Intake.SlideState.TRANSFERRING
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
                        new InstantCommand(() -> SpecimenScoringMode = false).alongWith(new OuttakeArmCommand(Outtake.ArmState.TRANSFERING))
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

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.drive.stop()),
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new IntakeArmCommand(Intake.ArmState.TRANSFERRING),
                        new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new OuttakeArmCommand(Outtake.ArmState.TRANSFERING)
                )
        );

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.loop();
            robot.write();
            multipleTelemetry.addLine("Initializing...");
            multipleTelemetry.addData("HZ", 1E9 / (System.nanoTime() - loopStamp));
            loopStamp = System.nanoTime();
            multipleTelemetry.update();
        }

        multipleTelemetry.clear();

        FtcDashboard.getInstance().startCameraStream(robot.baseCam, 100);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new HorizontalSlidesCommand(Intake.SlideState.TRANSFERRING, false),
                        new VerticalSlidesCommand(Outtake.SlideState.DEFAULT, false)
                )
        );

        //robot.odometry.setPosition(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(90))));

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            double scale = (1.0 - MIN_MOTOR_POWER) * (1.0 - Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) + MIN_MOTOR_POWER;

            if (!robot.intake.adjusting) {
                robot.drive.moveRobot(
                        new Vector2d(Gamepad1.getLeftX(), Gamepad1.getLeftY()).scale(scale),
                        new Vector2d(Gamepad1.getRightX(), Gamepad1.getRightY()).scale(scale),
                        robot.odometry.getHeading()
                );
            }

            robot.read();
            robot.loop();
            robot.write();

            //robot.verticalSlidesActuator.write(Gamepad2.getLeftY());

            //telemetry.addData("ROBOT HEADING", robot.odometry.getHeading());
            multipleTelemetry.addData("INTAKE SLIDES POS", robot.horizontalSlideActuator.getPosition());
            //multipleTelemetry.addData("INTAKE SLIDES POWER", robot.verticalSlidesActuator.power);
            //telemetry.addData("POS", robot.drive.sample);
            //telemetry.addData("POWER", robot.drive.t.toString() + " " + robot.drive.h.toString());
            multipleTelemetry.addData("SAMPLE", robot.drive.sample);
            //telemetry.addData("FPS", robot.baseCam.getFps() + " " + robot.baseCam.getCurrentPipelineMaxFps());
            telemetry.addData("ROBOT POS", robot.odometry.getPosition().toString());
            telemetry.addData("OUTTAKE SLIDES POS", robot.verticalSlidesActuator.profileOutput + " " + robot.verticalSlidesActuator.getPosition());
            //telemetry.addData("SAMPLE ANGLE", robot.sampleAlignmentPipeline.getSampleAngle());
            //telemetry.addData("INCH", robot.outtakeClawSensor.getDistance(DistanceUnit.INCH));
            //telemetry.addData("current", PandaRobot.getInstance().controlHub.getLynxModule().getCurrent(CurrentUnit.AMPS));
            multipleTelemetry.addData("HZ", 1E9 / (System.nanoTime() - loopStamp));
            loopStamp = System.nanoTime();
            multipleTelemetry.update();
        }

        robot.baseCam.stopStreaming();
        robot.baseCam.closeCameraDevice();
    }
}