package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystem.CameraSystems.SampleAlignmentPipeline;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Robot {
    // HUBS
    private HardwareMap hardwareMap;
    public RevHub controlHub, expansionHub;

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
    public HuskyLens huskyLens;
    public OpenCvCamera logitechCam;
    public SampleAlignmentPipeline sampleAlignmentPipeline;

    private static Robot instance = null;
    private ArrayList<Subsystem> subsystems;
    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    public void initialize(HardwareMap map) {
        hardwareMap = map;

        controlHub = new RevHub(hardwareMap, "Control Hub");
        //expansionHub = new RevHub(hardwareMap, "Expansion Hub 2");

        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        //expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        updateBulkCache();

        drive = new Mecanum();
        intake = new Intake();
        outtake = new Outtake();
        subsystems = new ArrayList<>();

        //subsystems.add(drive);
        subsystems.add(intake);
        //subsystems.add(outtake);

        /*odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odometry.setOffsets(-100, 100);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);*/

        if (Globals.opMode == Globals.RobotOpMode.AUTO) {
            //huskyLens = hardwareMap.get(HuskyLens.class, "h");
        }
        else {
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
    }

    public void read() {
        updateBulkCache();
        for (Subsystem s : subsystems) s.read();
    }

    public void loop() {
        if (Globals.opMode == Globals.RobotOpMode.AUTO) {
            //odometry.update();
        } else {
            //odometry.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        }
        for (Subsystem s : subsystems) s.loop();
    }

    public void write() {
        for (Subsystem s : subsystems) s.write();
    }

    public void updateBulkCache() {
        controlHub.updateBulkData();
        //expansionHub.updateBulkData();
    }
}
