package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.CycleSubCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.GoToBasketCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreGroundSamplesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

import java.util.concurrent.TimeUnit;

@Autonomous (name = "blue basket auto")
public class BlueBasketAuto extends LinearOpMode {
    private final PandaRobot robot = PandaRobot.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    GoToBasketCommand preload = new GoToBasketCommand();
    ScoreGroundSamplesCommand first = new ScoreGroundSamplesCommand(1);
    ScoreGroundSamplesCommand second = new ScoreGroundSamplesCommand(2);
    ScoreGroundSamplesCommand third = new ScoreGroundSamplesCommand(3);
    CycleSubCommand sub = new CycleSubCommand();
    private enum State {
        PRELOAD,
        FIRST_SAMPLE,
        SECOND_SAMPLE,
        THIRD_SAMPLE,
        SUBMERSIBLE,
        FINISHED
    }
    State state = State.PRELOAD;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Globals.opMode = Globals.RobotOpMode.AUTO;
        Globals.alliance = Globals.RobotAlliance.BLUE;
        Globals.targetingColorPiece = false;

        robot.initialize(hardwareMap);

        robot.odometry.setPosition(new Pose2d(0, 0, Math.toRadians(90)));

        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        robot.reset(false);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new WaitCommand(3000),
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED)
                )
        );

        while (opModeInInit()) {
            robot.update();
            telemetry.addLine("Initializing...");
            telemetry.update();
        }

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
                        CommandScheduler.getInstance().schedule(second);
                        state = State.SECOND_SAMPLE;
                    }
                    break;
                case SECOND_SAMPLE:
                    if (!CommandScheduler.getInstance().isScheduled(second)) {
                        CommandScheduler.getInstance().schedule(third);
                        state = State.THIRD_SAMPLE;
                    }
                    break;
                case THIRD_SAMPLE:
                    if (!CommandScheduler.getInstance().isScheduled(third)) {
                        CommandScheduler.getInstance().schedule(sub);
                        state = State.SUBMERSIBLE;
                    }
                    break;
                case SUBMERSIBLE:
                    if (!CommandScheduler.getInstance().isScheduled(sub)) {
                        state = State.FINISHED;
                    }
                case FINISHED:
                    break;
            }

            telemetry.addData("ACTION ", state.toString());
            telemetry.addData("RUNTIME", timer.time(TimeUnit.MILLISECONDS) / 1000.0);
            telemetry.update();
        }
    }
}
