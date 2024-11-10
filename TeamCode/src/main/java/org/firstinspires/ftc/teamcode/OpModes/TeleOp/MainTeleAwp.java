package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.Robots.Globals;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.GrabSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.RetractOuttakeArm;
import org.firstinspires.ftc.teamcode.Schedule.TeleOpCommand.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose;

@TeleOp (name="main tele op")
public class MainTeleAwp extends AbstractTeleOp {
    HuaHua robot;
    final double MIN_MOTOR_POWER = 0.4;
    long loopStamp = 0L;
    @Override
    public AbstractRobot instantiateRobot() {
        robot = new HuaHua(this);

        return robot;
    }

    @Override
    public void onInit() {
        robot.imu.resetYaw();

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

        // transfer sample
        /*robot.gamepad2.getGamepadButton(GamepadKeys.Button.X).
                    whenPressed(
                            new TransferSampleCommand(robot).whenFinished(() -> robot.gamepad2.gamepad.rumble(150))
                    );*/

        /*robot.gamepad2.getGamepadButton(GamepadKeys.Button.Y).
                    whenPressed(
                            new RetractOuttakeArm(robot).whenFinished(() -> robot.gamepad2.gamepad.rumble(150))
                    );*/

        CommandScheduler.getInstance().schedule(new IntakeArmCommand(robot, robot.intake.armState));
        CommandScheduler.getInstance().schedule(new InstantCommand(() -> robot.intake.resetClawRotation()));
        CommandScheduler.getInstance().schedule(new IntakeClawCommand(robot, robot.intake.clawState));
    }

    @Override
    public void onDriverUpdate() {
        CommandScheduler.getInstance().run();

        // controls driving
        double scale = (1.0 - MIN_MOTOR_POWER) * (1.0 - robot.gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) + MIN_MOTOR_POWER;

        robot.drive.calculatePower(new Pose(
                        robot.gamepad1.getLeftX() * scale, -robot.gamepad1.getLeftY() * scale, robot.gamepad1.getRightX() * scale),
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );


        if (robot.intake.armState == Intake.ArmState.INTAKING) {
            // rotate claw and move slides freely when intaking
            robot.intake.rotateClaw(robot.gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), robot.gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            //robot.intake.moveSlides(robot.gamepad2.getLeftY());
        }

        // go to scoring mode
        /*if (robot.outtake.armState == Outtake.ArmState.SCORING) {
            robot.outtake.updateClawState(robot.gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).get() ? Outtake.ClawState.OPENED : Outtake.ClawState.CLOSED);
            robot.outtake.moveSlides(robot.gamepad2.getLeftY());
        }*/
        robot.telemetry.addData("hz", 1000000000 / (System.nanoTime() - loopStamp));
        loopStamp = System.nanoTime();
        robot.telemetry.update();
    }

    @Override
    public void onStop() {

    }

}