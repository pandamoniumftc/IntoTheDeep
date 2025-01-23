package org.firstinspires.ftc.teamcode.OpModes.AutoOp;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Schedule.AutoCommands.ScoreSpecimenPreloadLeftSideCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@Autonomous (name = "blue basket auto")
public class BlueBasketAuto extends LinearOpMode {
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

        robot.odometry.setPosition(new Pose2D(DistanceUnit.MM, 0.0, 10.0, AngleUnit.RADIANS, 0));

        robot.read();
        robot.horizontalSlideActuator.setInitialPosition();
        robot.verticalSlidesActuator.setInitialPosition();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new IntakeClawCommand(Intake.ClawState.CLOSED),
                        new OuttakeArmCommand(Outtake.ArmState.SCORING_SAMPLE),
                        new OuttakeClawCommand(Outtake.ClawState.OPENED),
                        new IntakeArmCommand(Intake.ArmState.TRANSFERRING),
                        new WaitCommand(3000),
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
                        new ScoreSpecimenPreloadLeftSideCommand(),
                        //new ScoreFirstSampleCommand(),
                        //new ScoreSecondSampleCommand(),
                        //new ParkFirstLevelAscentCommand(),
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
            telemetry.addData("Runtime: ", timer.seconds());
            telemetry.update();

            loopTime = loop;
        }

        robot.baseCam.stopStreaming();
        robot.baseCam.closeCameraDevice();
    }
}
