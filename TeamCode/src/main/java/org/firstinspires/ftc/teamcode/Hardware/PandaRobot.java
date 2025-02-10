package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.VerticalSlidesCommand;
import org.firstinspires.ftc.teamcode.Subsystem.CameraSystems.SampleAlignmentPipeline;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
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
    public PandaServoActuator intakeArmActuator;
    public PandaServo intakeClawServo, intakeRotateClawServo, intakeRotateArmServo, intakeLightChain;

    // OUTTAKE
    public Outtake outtake;
    public PandaMotorActuator verticalSlidesActuator;
    public PandaServo outtakeClawServo, outtakePivotServo;
    public RevColorSensorV3 outtakeClawSensor;

    // CAMERA
    public OpenCvWebcam baseCam;
    public SampleAlignmentPipeline sampleAlignmentPipeline;

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

        drive = new Mecanum();
        intake = new Intake();
        outtake = new Outtake();

        voltageTimer = new ElapsedTime();

        outtakeClawSensor = hardwareMap.get(RevColorSensorV3.class, "claw");

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometry.setOffsets(-55, 110); // -55, 110
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        baseCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        sampleAlignmentPipeline = new SampleAlignmentPipeline();
        baseCam.setPipeline(sampleAlignmentPipeline);
        baseCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                baseCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
                //baseCam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                //baseCam.getExposureControl().setExposure(2500, TimeUnit.MICROSECONDS); // 0 - 500000 microseconds : 2500
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
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
        intake.loop();
        outtake.loop();
    }

    public void write() {
        drive.write();
        intake.write();
        outtake.write();
    }

    public void reset() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(
                new HorizontalSlidesCommand(Intake.SlideState.TRANSFERRING, false),
                new VerticalSlidesCommand(Outtake.SlideState.DEFAULT, false),
                new IntakeClawCommand(Intake.ClawState.CLOSED),
                new IntakeArmCommand(Intake.ArmState.DEFAULT),
                new OuttakeClawCommand(Outtake.ClawState.OPENED),
                new OuttakeArmCommand(Outtake.ArmState.TRANSFERING)
        );
    }
}
