package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreFirstSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreSecondSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreSpecimenPreloadLeftSideCommand;
import org.firstinspires.ftc.teamcode.Schedule.DriveCommand.PositionCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@Autonomous(name = "test")
public class Test extends LinearOpMode {
    private final PandaRobot robot = PandaRobot.getInstance();
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
        //robot.verticalSlidesActuator.setInitialPosition();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new OuttakeArmCommand(Outtake.ArmState.TRANSFERING),
                        new IntakeArmCommand(Intake.ArmState.TRANSFERRING)
                        /*new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new WaitCommand(3000),
                        new IntakeArmCommand(Intake.ArmState.TRANSFERRING),
                        new OuttakeClawCommand(Outtake.ClawState.CLOSED)*/
                )
        );

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
            telemetry.addLine("Initializing...");
            robot.read();
            robot.loop();
            robot.write();
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(new Pose2d(0, 100, new Rotation2d(0))),
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
            telemetry.addData("pos ", robot.odometry.getPosition().toString());
            telemetry.addData("t ", robot.drive.t);
            telemetry.addData("Runtime: ", timer.seconds());
            telemetry.update();

            loopTime = loop;
        }

        robot.oldAssCam.stopStreaming();
        robot.oldAssCam.closeCameraDevice();
    }
}
