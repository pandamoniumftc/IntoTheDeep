package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.CycleSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ParkObservationZone;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.PushSampleIntoZoneCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreSpecimenPreloadRightSideCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

import java.time.zone.ZoneRules;

@Autonomous(name = "blue specimen auto")
public class BlueSpecimenAuto extends LinearOpMode {
    private final PandaRobot robot = PandaRobot.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    private double loopTime = 0.0;
    PushSampleIntoZoneCommand push = new PushSampleIntoZoneCommand();
    CycleSpecimenCommand firstSpec = new CycleSpecimenCommand(1);
    CycleSpecimenCommand secondSpec = new CycleSpecimenCommand(2);
    CycleSpecimenCommand thirdSpec = new CycleSpecimenCommand(3);
    CycleSpecimenCommand fourthSpec = new CycleSpecimenCommand(4);
    ParkObservationZone zone = new ParkObservationZone();

    private enum State {
        PUSH,
        FIRST_SPEC,
        SECOND_SPEC,
        THIRD_SPEC,
        FOURTH_SPEC,
        PARK,
        FINISHED
    }
    State state = State.PUSH;
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Globals.opMode = Globals.RobotOpMode.AUTO;
        Globals.alliance = Globals.RobotAlliance.BLUE;

        robot.initialize(hardwareMap);

        robot.odometry.resetPosAndIMU();

        robot.read();
        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        robot.reset(false);

        while (opModeInInit()) {
            robot.update();
            telemetry.addLine("Initializing...");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(push);

        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            switch(state) {
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
                    if (!CommandScheduler.getInstance().isScheduled(secondSpec)) {
                        CommandScheduler.getInstance().schedule(thirdSpec);
                        state = State.THIRD_SPEC;
                    }
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
                    Globals.pose = robot.drive.currentPosition;
                    break;
            }


            double loop = System.nanoTime();
            telemetry.addData("state ", state);
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addLine(robot.odometry.getPosition().toString());
            telemetry.addData("Runtime: ", timer.seconds());
            telemetry.update();

            loopTime = loop;
        }
    }
}
