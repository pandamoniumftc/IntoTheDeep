package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Subsystem.CameraSystems.SampleAlignmentPipeline;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Robot {
    // HUBS
    private HardwareMap hardwareMap;
    public RevHub controlHub, expansionHub;
    ElapsedTime voltageTimer;
    public double voltage = 12.0;

    // DRIVETRAIN
    public Mecanum drive;
    public Motor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;

    // LOCALIZER
    public GoBildaPinpointDriver odometry;

    // INTAKE
    public Intake intake;
    public MotorActuator horizontalSlideActuator;
    public Servo intakeClawServo, intakeRotateServo, intakeLeftWristServo, intakeRightWristServo, intakeLeftElbowServo, intakeRightElbowServo;

    // OUTTAKE
    public Outtake outtake;
    public MotorActuator verticalSlidesActuator;
    public Servo outtakeClawServo, outtakePivotServo;

    // CAMERA
    public OpenCvCamera logitechCam;
    public SampleAlignmentPipeline sampleAlignmentPipeline;
    private static Robot instance = null;
    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    public void initialize(HardwareMap map) {
        hardwareMap = map;

        controlHub = new RevHub(hardwareMap, "Control Hub");
        expansionHub = new RevHub(hardwareMap, "Expansion Hub 2");

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        updateBulkCache();

        drive = new Mecanum();
        intake = new Intake();
        outtake = new Outtake();

        voltageTimer = new ElapsedTime();

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometry.setOffsets(-55, 110);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        logitechCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        sampleAlignmentPipeline = new SampleAlignmentPipeline();
        logitechCam.setPipeline(sampleAlignmentPipeline);
        logitechCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                logitechCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public void read() {
        updateBulkCache();
        intake.read();
        outtake.read();
    }

    public void loop() {
        if (Globals.opMode == Globals.RobotOpMode.AUTO) {
            odometry.update();
        }
        else {
            odometry.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        }

        if (voltageTimer.time(TimeUnit.SECONDS) > 10) {
            voltage = controlHub.module.getInputVoltage(VoltageUnit.VOLTS);
            voltageTimer.reset();
        }
        intake.loop();
        outtake.loop();
    }

    public void write() {
        intake.write();
        outtake.write();
        drive.write();
    }

    public void updateBulkCache() {
        controlHub.updateBulkData();
        expansionHub.updateBulkData();
    }
}
