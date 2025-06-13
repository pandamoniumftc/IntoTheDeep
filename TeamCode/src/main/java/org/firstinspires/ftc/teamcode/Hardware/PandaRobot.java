package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ExtendIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.GrabObjectCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.HighBasketCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.HighChamberCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ResetVerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.RetractIntakeCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSampleCommand;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

public class PandaRobot {
    // HUBS
    private HardwareMap hardwareMap;
    public PandaHub controlHub, expansionHub;
    public LynxModule.BulkData bulkData;
    public HashMap<Sensors, Integer[]> sensorValues;
    ElapsedTime voltageTimer;
    public double voltage = 12.0;
    public double currentReading;

    // DRIVETRAIN
    public PandaMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public Mecanum drive;

    // LOCALIZER
    public GoBildaPinpointDriver odometry;

    // INTAKE
    public Intake intake;
    public PandaMotorActuator horizontalSlideActuator;
    public PandaServo leftArmServo, rightArmServo, intakeClawServo, intakeRotateClawServo, intakeRotateArmServo, intakeLightChain;

    // OUTTAKE
    public Outtake outtake;
    public PandaMotorActuator verticalSlidesActuator;
    public PandaServo outtakeClawServo, outtakeElbowServo, outtakeWristServo;
    public AsyncRev2MSensor outtakeClawSensor;

    // CAMERA
    public Limelight3A limelight;

    // GAMEPADS
    public GamepadEx gamepad1, gamepad2;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static PandaRobot instance = null;
    public static PandaRobot getInstance() {
        if (instance == null) {
            instance = new PandaRobot();
        }
        return instance;
    }

    public void initialize(HardwareMap map) {
        hardwareMap = map;

        sensorValues = new HashMap<>();

        sensorValues.put(Sensors.HORIZONTAL_SLIDES, new Integer[]{0, 0});
        sensorValues.put(Sensors.VERTICAL_SLIDES, new Integer[]{0, 0});

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent() && LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber())) {
                controlHub = new PandaHub(hardwareMap, module);
            }
            else {
                expansionHub = new PandaHub(hardwareMap, module);
            }
        }

        drive = new Mecanum(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        PathLibrary.initialize();

        voltageTimer = new ElapsedTime();
    }

    public void initialize(HardwareMap map, Gamepad gamepad1, Gamepad gamepad2) {
        hardwareMap = map;

        sensorValues = new HashMap<>();

        sensorValues.put(Sensors.HORIZONTAL_SLIDES, new Integer[]{0, 0});
        sensorValues.put(Sensors.VERTICAL_SLIDES, new Integer[]{0, 0});

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent() && LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber())) {
                controlHub = new PandaHub(hardwareMap, module);
            }
            else {
                expansionHub = new PandaHub(hardwareMap, module);
            }
        }

        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);

        drive = new Mecanum(hardwareMap, this.gamepad1);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        voltageTimer = new ElapsedTime();

        /**
         * Extend horizontal slides to enter object detection mode
         * Retract horizontal slides to automatically transfer the object to the outtake claw
         */
        this.gamepad2.getGamepadButton(GamepadKeys.Button.B).
                toggleWhenPressed(
                        new ExtendIntakeCommand(),
                        new RetractIntakeCommand()
                );

        /**
         * Automatically adjust the robot position and horizontal slide to the random
         * detected object position and grabs it only if the horizontal slides are extended
         */
        this.gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new GrabObjectCommand());

        /**
         * Used to move samples to the observation zone by dropping the object in front of the robot
         */
        this.gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new OuttakeArmCommand(Outtake.ArmState.SCORING_SPECIMEN),
                                new WaitCommand(400),
                                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                                new WaitCommand(150),
                                new OuttakeArmCommand(Outtake.ArmState.TRANSFERRING)
                        ),
                        new InstantCommand(),
                        () -> PandaRobot.getInstance().outtake.slideState == Outtake.SlideState.DEFAULT
                )
        );

        /**
         * Raises the vertical slides to the high basket level
         */
        this.gamepad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new HighBasketCommand());

        /**
         * Raises the vertical slides to the high chamber level
         */
        this.gamepad2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new HighChamberCommand());

        /**
         * Simply drop the sample into the basket if at that level and resets the vertical slides
         * 1st press scores specimen on to the bar; 2nd press resets the vertical slides
         */
        this.gamepad2.getGamepadButton(GamepadKeys.Button.Y).
                whenPressed(
                        new ConditionalCommand(
                                new ConditionalCommand(
                                        new ScoreSampleCommand(),
                                        new ScoreSpecimenCommand(false),
                                        () -> outtake.slideState == Outtake.SlideState.HIGH_BASKET
                                ),
                                new InstantCommand(),
                                () -> outtake.slideState != Outtake.SlideState.DEFAULT
                        )
                );

        /**
         * Choose to detect only yellow samples
         */
        this.gamepad1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> Globals.targetingColorPiece = false));

        /**
         * Choose to only detect red samples and specimens
         */
        this.gamepad1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> Globals.targetingColorPiece = true),
                        new InstantCommand(() -> Globals.alliance = Globals.RobotAlliance.RED)
                )
        );

        /**
         * Choose to only detect blue samples and specimens
         */
        this.gamepad1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> Globals.targetingColorPiece = true),
                        new InstantCommand(() -> Globals.alliance = Globals.RobotAlliance.BLUE)
                )
        );

        /**
         * Enabling angle assist disables right joystick movement and uses PID to align robot to a certain angle
         * Disabling angle assist enables right joystick movement again
         */
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() -> drive.enableTargetAngle(0)));
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> drive.enableTargetAngle(90)));
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(() -> drive.enableTargetAngle(-90)));
        this.gamepad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() -> drive.disableTargetAngle()));
    }

    public void updateSubsystems() {
        bulkData = expansionHub.getLynxModule().getBulkData();
        sensorValues.put(Sensors.HORIZONTAL_SLIDES, new Integer[] {bulkData.getMotorCurrentPosition(0), bulkData.getMotorVelocity(0)});
        sensorValues.put(Sensors.VERTICAL_SLIDES, new Integer[] {bulkData.getMotorCurrentPosition(3), bulkData.getMotorVelocity(3)});

        if (voltageTimer.time(TimeUnit.SECONDS) > 10) {
            voltage = controlHub.getLynxModule().getInputVoltage(VoltageUnit.VOLTS);
            voltageTimer.reset();
        }

        drive.update();
        intake.update();
        outtake.update();
    }

    public void reset(boolean stopDriveTrain) {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(
                new HorizontalSlidesCommand(Intake.SlideState.TRANSFERRING, false),
                new VerticalSlidesCommand(Outtake.SlideState.DEFAULT, false),
                new IntakeClawCommand(Intake.ClawState.OPENED),
                new IntakeArmCommand(Intake.ArmState.TRANSFERRING),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new OuttakeArmCommand(Outtake.ArmState.TRANSFERRING)
        );
        if (stopDriveTrain) CommandScheduler.getInstance().schedule(new InstantCommand(() -> drive.stop()));
    }

    public void update() {
        CommandScheduler.getInstance().run();
        updateSubsystems();
    }
}
