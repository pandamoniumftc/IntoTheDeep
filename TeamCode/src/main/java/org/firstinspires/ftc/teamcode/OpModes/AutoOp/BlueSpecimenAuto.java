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
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.PushSampleIntoZoneCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreSpecimenPreloadRightSideCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@Autonomous(name = "blue specimen auto")
public class BlueSpecimenAuto extends LinearOpMode {
    private final Robot robot = Robot.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    private double loopTime = 0.0;
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

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new IntakeArmCommand(Intake.ArmState.DEFAULT),
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE),
                        new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new WaitCommand(2000),
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED)
                )
        );

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
            telemetry.addLine("Initializing...");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(new Pose2d(new Translation2d(300, 0), new Rotation2d(0))),
                        new WaitUntilCommand(() -> timer.seconds() > 30)
                )
        );

        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.loop();
            robot.write();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addLine(robot.odometry.getPosition().toString());
            telemetry.addData("Runtime: ", timer.seconds());
            telemetry.update();

            loopTime = loop;
        }

        robot.logitechCam.stopStreaming();
        robot.logitechCam.closeCameraDevice();
    }
}
