package org.firstinspires.ftc.teamcode.Hardware;

import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
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
import java.util.concurrent.TimeUnit;

public class PandaRobot {
    // HUBS
    private HardwareMap hardwareMap;
    public PandaHub controlHub, expansionHub;
    PhotonLynxVoltageSensor voltageSensor;
    ElapsedTime voltageTimer;
    public double voltage = 12.0;

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

    // CAMERA
    public OpenCvWebcam baseCam;
    public long minimumExposure;
    public SampleAlignmentPipeline sampleAlignmentPipeline;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private ArrayList<Subsystem> subsystems;
    private static PandaRobot instance = null;
    public static PandaRobot getInstance() {
        if (instance == null) {
            instance = new PandaRobot();
        }
        return instance;
    }

    public void initialize(HardwareMap map) {
        hardwareMap = map;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.isParent() && LynxConstants.isEmbeddedSerialNumber(module.getSerialNumber())) {
                controlHub = new PandaHub(map, module);
            }
            else {
                expansionHub = new PandaHub(map, module);
            }
        }

        voltageSensor = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();

        drive = new Mecanum();
        intake = new Intake();
        outtake = new Outtake();
        subsystems = new ArrayList<>();

        subsystems.add(drive);
        subsystems.add(intake);
        subsystems.add(outtake);

        voltageTimer = new ElapsedTime();

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometry.setOffsets(-55, 110);
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
                baseCam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
                baseCam.getExposureControl().setMode(ExposureControl.Mode.Manual);
                baseCam.getExposureControl().setExposure(200, TimeUnit.MICROSECONDS); // 0 - 500000 microseconds
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void read() {
        controlHub.getLynxModule().clearBulkCache();
        expansionHub.getLynxModule().clearBulkCache();

        if (Globals.opMode == Globals.RobotOpMode.AUTO) {
            odometry.update();
        }
        else {
            odometry.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        }

        if (voltageTimer.time(TimeUnit.SECONDS) > 10) {
            voltage = voltageSensor.getCachedVoltage();
            voltageTimer.reset();
        }

        for (Subsystem system : subsystems) system.read();
    }

    public void loop() {
        for (Subsystem system : subsystems) system.loop();
    }

    public void write() {
        for (Subsystem system : subsystems) system.write();
    }
}
