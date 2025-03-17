package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.GoToBasketCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreFirstSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreSecondSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreThirdSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.SplineCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;
import org.firstinspires.ftc.teamcode.Util.Spline;

@Autonomous(name = "red basket auto")
public class RedBasketAuto extends LinearOpMode {
    private final PandaRobot robot = PandaRobot.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    private double loopTime = 0.0;
    GoToBasketCommand preload = new GoToBasketCommand();
    ScoreFirstSampleCommand first = new ScoreFirstSampleCommand();
    ScoreSecondSampleCommand second = new ScoreSecondSampleCommand();
    ScoreThirdSampleCommand third = new ScoreThirdSampleCommand();
    private enum State {
        PRELOAD,
        FIRST_SAMPLE,
        SECOND_SAMPLE,
        THIRD_SAMPLE,
        SUBMERSIBLE,
        FINISHED
    }
    State state = State.PRELOAD;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Globals.opMode = Globals.RobotOpMode.AUTO;
        Globals.alliance = Globals.RobotAlliance.RED;

        robot.initialize(hardwareMap);

        robot.odometry.resetPosAndIMU();

        robot.read();
        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new OuttakeArmCommand(Outtake.ArmState.TRANSFERRING),
                        new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new IntakeArmCommand(Intake.ArmState.DEFAULT),
                        new WaitCommand(3000),
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED)
                )
        );

        while (opModeInInit()) {
            robot.update();
            telemetry.addLine("Initializing...");
            telemetry.update();
        }

        robot.odometry.setPosition(new Pose2d(0.0, 0.0, Math.toRadians(90)));

        CommandScheduler.getInstance().schedule(preload);

        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            switch(state) {
                case PRELOAD:
                    if (!CommandScheduler.getInstance().isScheduled(preload)) {
                        CommandScheduler.getInstance().schedule(first);
                        state = State.FIRST_SAMPLE;
                    }
                    break;
                case FIRST_SAMPLE:
                    if (!CommandScheduler.getInstance().isScheduled(first)) {
                        if (robot.intake.slideState == Intake.SlideState.TRANSFERRING || robot.intake.retries == 1) {
                            CommandScheduler.getInstance().schedule(second);
                            state = State.SECOND_SAMPLE;
                        }
                        else {
                            robot.intake.retries++;
                            CommandScheduler.getInstance().schedule(first);
                        }
                    }
                    break;
                case SECOND_SAMPLE:
                    if (!CommandScheduler.getInstance().isScheduled(second)) {
                        if (robot.intake.slideState == Intake.SlideState.TRANSFERRING || robot.intake.retries == 1) {
                            //CommandScheduler.getInstance().schedule(third);
                            state = State.FINISHED;
                        }
                        else {
                            robot.intake.retries++;
                            CommandScheduler.getInstance().schedule(second);
                        }
                    }
                    break;
                case THIRD_SAMPLE:
                    if (!CommandScheduler.getInstance().isScheduled(third)) {
                        if (robot.intake.slideState == Intake.SlideState.TRANSFERRING || robot.intake.retries == 1) {
                            state = State.FINISHED;
                        }
                        else {
                            robot.intake.retries++;
                            CommandScheduler.getInstance().schedule(third);
                        }
                    }
                    break;
                case FINISHED:
                    Globals.pose = robot.drive.currentPosition;
                    break;
            }

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("pos ", robot.odometry.getPosition().toString());
            telemetry.addData("Runtime: ", timer.seconds());
            telemetry.update();

            loopTime = loop;
        }
    }
}
