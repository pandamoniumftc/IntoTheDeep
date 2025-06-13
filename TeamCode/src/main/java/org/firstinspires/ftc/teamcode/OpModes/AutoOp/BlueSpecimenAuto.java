package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.CycleSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ParkObservationZone;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.PushSampleIntoZoneCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreSpecimenPreloadRightSideCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Util.Pose2d;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "blue specimen auto")
public class BlueSpecimenAuto extends LinearOpMode {
    private final PandaRobot robot = PandaRobot.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    ScoreSpecimenPreloadRightSideCommand preload = new ScoreSpecimenPreloadRightSideCommand();
    PushSampleIntoZoneCommand push = new PushSampleIntoZoneCommand();
    CycleSpecimenCommand firstSpec = new CycleSpecimenCommand(1);
    CycleSpecimenCommand secondSpec = new CycleSpecimenCommand(2);
    CycleSpecimenCommand thirdSpec = new CycleSpecimenCommand(3);
    CycleSpecimenCommand fourthSpec = new CycleSpecimenCommand(4);
    ParkObservationZone zone = new ParkObservationZone();

    private enum State {
        PRELOAD,
        PUSH,
        FIRST_SPEC,
        SECOND_SPEC,
        THIRD_SPEC,
        FOURTH_SPEC,
        PARK,
        FINISHED
    }
    State state = State.PRELOAD;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Globals.opMode = Globals.RobotOpMode.AUTO;
        Globals.alliance = Globals.RobotAlliance.BLUE;
        Globals.targetingColorPiece = true;

        robot.initialize(hardwareMap);

        robot.odometry.setPosition(new Pose2d(0, 0, Math.toRadians(0)));

        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        robot.reset(false);

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new WaitCommand(3000),
                new OuttakeClawCommand(Outtake.ClawState.CLOSED)));

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
                        CommandScheduler.getInstance().schedule(push);
                        state = State.PUSH;
                    }
                    break;
                case PUSH:
                    if (!CommandScheduler.getInstance().isScheduled(push)) {
                        CommandScheduler.getInstance().schedule(firstSpec);
                        state = State.FIRST_SPEC;
                    }
                    break;
                case FIRST_SPEC:
                    if (!CommandScheduler.getInstance().isScheduled(firstSpec)) {
                        CommandScheduler.getInstance().schedule(secondSpec);
                        state = State.SECOND_SPEC;
                    }
                    break;
                case SECOND_SPEC:
                    /*if (!CommandScheduler.getInstance().isScheduled(secondSpec)) {
                        CommandScheduler.getInstance().schedule(thirdSpec);
                        state = State.THIRD_SPEC;
                    }*/
                    break;
                case THIRD_SPEC:
                    if (!CommandScheduler.getInstance().isScheduled(thirdSpec)) {
                        CommandScheduler.getInstance().schedule(zone);
                        state = State.PARK;
                    }
                    break;
                case PARK:
                    if (!CommandScheduler.getInstance().isScheduled(zone)) {
                        state = State.FINISHED;
                    }
                case FINISHED:
                    break;
            }

            telemetry.addData("ACTION ", state.toString());
            telemetry.addData("RUNTIME: ", timer.time(TimeUnit.MILLISECONDS)/1000.0);
            telemetry.update();
        }
    }
}
