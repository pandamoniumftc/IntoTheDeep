package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Schedule.MacroCommand.ResetVerticalSlidesCommand;
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
    public double current;

    // DRIVETRAIN
    public Mecanum drive;
    public PandaMotor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;

    // LOCALIZER
    public GoBildaPinpointDriver odometry;

    // INTAKE
    public Intake intake;
    public PandaMotorActuator horizontalSlideActuator;
    public PandaServo leftArmServo, rightArmServo, intakeClawServo, intakeRotateClawServo, intakeRotateArmServo, intakeLightChain;

    // OUTTAKE
    public Outtake outtake;
    public PandaMotorActuator verticalSlidesActuator;
    public PandaServo outtakeClawServo, outtakePivotServo;
    public AsyncRev2MSensor outtakeClawSensor;

    // CAMERA
    public Limelight3A limelight;

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

        voltageTimer = new ElapsedTime();
    }

    public void read() {
        bulkData = expansionHub.getLynxModule().getBulkData();
        sensorValues.put(Sensors.HORIZONTAL_SLIDES, new Integer[] {bulkData.getMotorCurrentPosition(0), bulkData.getMotorVelocity(0)});
        sensorValues.put(Sensors.VERTICAL_SLIDES, new Integer[] {bulkData.getMotorCurrentPosition(3), bulkData.getMotorVelocity(3)});

        if (voltageTimer.time(TimeUnit.SECONDS) > 10) {
            voltage = controlHub.getLynxModule().getInputVoltage(VoltageUnit.VOLTS);
            voltageTimer.reset();
        }

        drive.read();
        intake.read();
        outtake.read();
    }

    public void loop() {
        drive.loop();
        intake.loop();
        outtake.loop();
    }

    public void write() {
        drive.write();
        intake.write();
        outtake.write();
    }

    public void reset(boolean stopDriveTrain) {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(
                new HorizontalSlidesCommand(Intake.SlideState.TRANSFERRING, false),
                new ResetVerticalSlidesCommand(false),
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                new IntakeArmCommand(Intake.ArmState.DEFAULT),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new OuttakeArmCommand(Outtake.ArmState.CHECKING)
        );
        if (stopDriveTrain) CommandScheduler.getInstance().schedule(new InstantCommand(() -> drive.stop()));
    }

    public void update() {
        CommandScheduler.getInstance().run();
        read();
        loop();
        write();
    }
}
