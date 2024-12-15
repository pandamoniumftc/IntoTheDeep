package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@TeleOp (name="main tele op")
public class MainTeleAwp extends CommandOpMode {
    Robot robot = Robot.getInstance();
    GamepadEx Gamepad1, Gamepad2;
    final double MIN_MOTOR_POWER = 0.4;
    long loopStamp = 0L;
    boolean SpecimenScoringMode = false;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.telemetryEnable = true;

        Globals.opMode = Globals.RobotOpMode.TELEOP;

        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        robot.initialize(hardwareMap);

        // extend and retracts intake arm
        Gamepad2.getGamepadButton(GamepadKeys.Button.B).
                toggleWhenPressed(
                        new ExtendIntakeCommand(),
                        new GrabSampleCommand()
                );

        // retracts slides and transfers object
        Gamepad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new TransferSampleCommand());

        // switches between scoring modes
        Gamepad2.getGamepadButton(GamepadKeys.Button.START).
                toggleWhenPressed(
                        new InstantCommand(() -> SpecimenScoringMode = true),
                        new InstantCommand(() -> SpecimenScoringMode = false)
                );

        robot.horizontalSlideActuator.setInitialPosition();
        //robot.verticalSlidesActuator.setInitialPosition();

        robot.intake.updateClawState(Intake.ClawState.CLOSED);
        robot.intake.updateArmState(Intake.ArmState.DEFAULT);
        //robot.outtake.updateArmState(Outtake.ArmState.TRANSFERING);
        //robot.outtake.updateClawState(Outtake.ClawState.OPENED);

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.controlHub.updateVoltage();
        robot.read();
        robot.loop();
        robot.write();

        double scale = (1.0 - MIN_MOTOR_POWER) * (1.0 - Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) + MIN_MOTOR_POWER;

        /*robot.drive.moveRobot(
                new Vector2d(Gamepad1.getLeftX(), Gamepad1.getLeftY()).scale(scale),
                new Vector2d(Gamepad1.getRightX(), Gamepad1.getRightY()).scale(scale),
                robot.odometry.getHeading()
        );*/

        if (robot.outtake.getSlidesPosition() > 4 && robot.outtake.armState == Outtake.ArmState.TRANSFERING && !SpecimenScoringMode) {
            CommandScheduler.getInstance().schedule(new OuttakeArmCommand(Outtake.ArmState.SCORING));
        }
        else if (robot.outtake.getSlidesPosition() < 4 && robot.outtake.armState == Outtake.ArmState.SCORING && !SpecimenScoringMode) {
            CommandScheduler.getInstance().schedule(new OuttakeArmCommand(Outtake.ArmState.TRANSFERING));
        }

        if (robot.outtake.armState == Outtake.ArmState.TRANSFERING && SpecimenScoringMode) {
            CommandScheduler.getInstance().schedule(new OuttakeArmCommand(Outtake.ArmState.SCORING));
        }

        if (robot.outtake.armState == Outtake.ArmState.SCORING) {
            robot.outtake.updateClawState(Gamepad2.getGamepadButton(GamepadKeys.Button.Y).get() ? Outtake.ClawState.OPENED : Outtake.ClawState.CLOSED);
        }

        //robot.outtake.moveSlides(Gamepad2.getLeftY());

        if (Globals.telemetryEnable) {
            //telemetry.addData("ROBOT HEADING", robot.odometry.getHeading());
            telemetry.addData("HORIZONTAL SLIDE POS", robot.horizontalSlideActuator.getPosition() + " MM; " + robot.horizontalSlideActuator.profileOutput);
            //telemetry.addData("OUTTAKE SLIDES POS", (15.5 + robot.outtake.getSlidesPosition()) + " IN");
            //telemetry.addData("SAMPLE ANGLE", robot.sampleAlignmentPipeline.getSampleAngle());
            telemetry.addData("HZ", 1E9 / (System.nanoTime() - loopStamp));
            loopStamp = System.nanoTime();
            telemetry.update();
        }
    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        //robot.logitechCam.stopStreaming();
        //robot.logitechCam.closeCameraDevice();
    }
}