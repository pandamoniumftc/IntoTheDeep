package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@TeleOp (name="main tele op")
public class MainTeleAwp extends LinearOpMode {
    PandaRobot robot = PandaRobot.getInstance();
    GamepadEx Gamepad1, Gamepad2;
    final double MIN_MOTOR_POWER = 0.3;
    long loopStamp = 0L;
    boolean SpecimenScoringMode = false;
    double pos = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        Globals.opMode = Globals.RobotOpMode.TELEOP;
        Globals.alliance = Globals.RobotAlliance.BLUE;

        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        robot.initialize(hardwareMap);

        robot.odometry.resetPosAndIMU();

        robot.read();
        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        // extend and retracts intake arm
        Gamepad2.getGamepadButton(GamepadKeys.Button.B).
                toggleWhenPressed(
                        new ConditionalCommand(
                                new ExtendIntakeCommand(),
                                new WaitCommand(0),
                                () -> !SpecimenScoringMode
                        ),
                        new ConditionalCommand(
                                new GrabSampleCommand(),
                                new WaitCommand(0),
                                () -> !SpecimenScoringMode
                        )
                );

        // retracts slides and transfers object
        Gamepad2.getGamepadButton(GamepadKeys.Button.A).
                whenPressed(
                        new ConditionalCommand(
                                new TransferSampleCommand(),
                                new WaitCommand(0),
                                () -> !SpecimenScoringMode
                        )
                );

        // switches between scoring modes
        /*Gamepad2.getGamepadButton(GamepadKeys.Button.BACK).
                toggleWhenPressed(
                        new InstantCommand(() -> SpecimenScoringMode = true).alongWith(new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN)),
                        new InstantCommand(() -> SpecimenScoringMode = false).alongWith(new OuttakeArmCommand(Outtake.ArmState.TRANSFERING))
                );

        Gamepad2.getGamepadButton(GamepadKeys.Button.X).
                toggleWhenPressed(
                        new ConditionalCommand(
                                new VerticalSlidesCommand(2000, true).andThen(new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN)),
                                new VerticalSlidesCommand(3750, true).andThen(new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE)),
                                () -> SpecimenScoringMode
                        )
                );

        Gamepad2.getGamepadButton(GamepadKeys.Button.Y).
                whenPressed(
                        new VerticalSlidesCommand(0, false).andThen(
                                new ConditionalCommand(
                                        new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN),
                                        new OuttakeArmCommand(Outtake.ArmState.TRANSFERING),
                                        () -> SpecimenScoringMode
                                )
                        )
                );

        Gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).
                toggleWhenPressed(
                        new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED)
                );

        Gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).
                whenPressed(
                        new ConditionalCommand(
                                new ScoreSpecimenCommand(),
                                new WaitCommand(0),
                                () -> SpecimenScoringMode
                        )
                );*/

        telemetry.addLine("Robot Initialized");
        telemetry.update();
        telemetry.clear();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new HorizontalSlidesCommand(Intake.SlideState.TRANSFERRING, false),
                        new IntakeArmCommand(Intake.ArmState.DEFAULT),
                        new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new OuttakeArmCommand(Outtake.ArmState.TRANSFERING)
                )
        );

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.loop();
            robot.write();
            telemetry.addLine("Initializing...");
            telemetry.update();
        }

        telemetry.clear();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.loop();
            robot.write();

            double scale = (1.0 - MIN_MOTOR_POWER) * (1.0 - Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) + MIN_MOTOR_POWER;

            //pos += Gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 1E-5;
            //pos -= Gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 1E-5;

            //robot.intakeRotateClawServo.setPosition(pos);

            //robot.outtake.write(Gamepad2.getLeftY());

            robot.drive.moveRobot(
                    new Vector2d(Gamepad1.getLeftX(), Gamepad1.getLeftY()).scale(scale),
                    new Vector2d(Gamepad1.getRightX(), Gamepad1.getRightY()).scale(scale),
                    robot.odometry.getHeading()
            );
            //robot.drive.test(Gamepad1);

            telemetry.addData("ROBOT HEADING", robot.odometry.getHeading());
            telemetry.addData("INTAKE SLIDES POS", robot.horizontalSlideActuator.getPosition());
            telemetry.addData("POS", robot.drive.sample);
            //telemetry.addData("POWER", robot.drive.t.toString() + " " + robot.drive.h.toString());
            //telemetry.addData("ROBOT POS", robot.odometry.getPosition().toString());
            telemetry.addData("OUTTAKE SLIDES POS", robot.verticalSlidesActuator.profileOutput + " " + robot.verticalSlidesActuator.getPosition());
            //telemetry.addData("SAMPLE ANGLE", robot.sampleAlignmentPipeline.getSampleAngle());
            //telemetry.addData("DISTANCE", robot.sampleSensor.getDistance(DistanceUnit.MM));
            //telemetry.addData("POWER", robot.drive.print());
            //telemetry.addData("PWM", robot.intakeRotateArmServo.getServo().isPwmEnabled() + " " + robot.intakeRotateClawServo.getServo().isPwmEnabled());
            telemetry.addData("HZ", 1E9 / (System.nanoTime() - loopStamp));
            loopStamp = System.nanoTime();
            telemetry.update();
        }

        //robot.logitechCam.stopStreaming();
        //robot.logitechCam.closeCameraDevice();
    }
}