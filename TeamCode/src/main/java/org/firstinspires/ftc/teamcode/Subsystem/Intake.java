package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static com.qualcomm.robotcore.util.Range.scale;

import static java.lang.Math.PI;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.AsyncRev2MSensor;
import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotor;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotorActuator;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Hardware.PandaServo;
import org.firstinspires.ftc.teamcode.Hardware.PandaServoActuator;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;
import org.firstinspires.ftc.teamcode.Schedule.SubsystemCommand.HorizontalSlidesCommand;

import java.util.List;

public class Intake extends Subsystem {
    private final PandaRobot robot;
    public enum ClawState {
        OPENED,
        CLOSED
    }
    public enum ArmState {
        GRABBING,
        TRANSFERRING,
        DEFAULT,
        SCANNING,
        ROTATE
    }
    public enum SlideState {
        TRANSFERRING,
        GRABBING_SAMPLE
    }
    public ClawState clawState;
    public ArmState armState;
    public SlideState slideState;
    public double clawPosition, tx, ty, alliance, current, retries;
    public Intake(HardwareMap hardwareMap) {
        robot = PandaRobot.getInstance();

        robot.horizontalSlideActuator = new PandaMotorActuator(
                new PandaMotor[] {
                        robot.expansionHub.getMotor(0)
                                .setDirection(DcMotorSimple.Direction.FORWARD)
                                .setCacheTolerance(0.02)
                },
                Sensors.HORIZONTAL_SLIDES,
                false)
                .setPIDController(0.012, 0.0, 0.0)
                .setLimits(0.0, 175)
                .setMotionProfile(350, 1200)
                .setTolerance(5)
                .setPowerThreshold(0.1);

        robot.leftArmServo = robot.expansionHub.getServo(5).setDirection(Servo.Direction.FORWARD); // PREVIOUSLY EXPANSION 3
        robot.rightArmServo = robot.controlHub.getServo(5).setDirection(Servo.Direction.REVERSE); // PREVIOUSLY CONTROL 0
        robot.intakeClawServo = robot.controlHub.getServo(0); // PREVIOUSLY CONTROL 5
        robot.intakeRotateClawServo = robot.controlHub.getServo(3); // 0* = 0.445, 90* = 0.384, 180* = 0.327
        robot.intakeRotateArmServo = robot.expansionHub.getServo(4);
        robot.intakeLightChain = robot.expansionHub.getServo(3); // PREVIOUSLY EXPANSION 5

        robot.limelight = hardwareMap.get(Limelight3A.class, "ll");
        robot.limelight.start();
        robot.limelight.pipelineSwitch(2);

        alliance = Globals.alliance == Globals.RobotAlliance.BLUE ? 0 : 1;
    }

    @Override
    public void read() {
        robot.horizontalSlideActuator.read();
        if (slideState == SlideState.GRABBING_SAMPLE && armState == ArmState.SCANNING)  {
            robot.limelight.updatePythonInputs(alliance, 0, 0, 0, 0, 0, 0, 0);
            tx = robot.limelight.getLatestResult().getTx();
            ty = robot.limelight.getLatestResult().getTy();
            clawPosition = scale(robot.limelight.getLatestResult().getPythonOutput()[0], 0, 180, 0.445, 0.326);
        }
    }

    @Override
    public void loop() {
        robot.horizontalSlideActuator.loop();
    }

    @Override
    public void write() {
        robot.horizontalSlideActuator.write();
    }

    public void write(double power) {
        robot.horizontalSlideActuator.write(power);
    }

    public void updateClawState(ClawState state) {
        if (clawState == state) {
            return;
        }
        this.clawState = state;
        robot.intakeClawServo.setPosition((clawState == ClawState.OPENED) ? 0.72 : 0.54);
    }
    public void updateArmState(ArmState state) {
        if (armState == state) {
            return;
        }
        this.armState = state;
        switch (armState) {
            case GRABBING:
                setArmPosition(0.765);
                break;
            case TRANSFERRING:
                setArmPosition(0.465);
                robot.intakeRotateArmServo.setPosition(0.395); // 0.385
                robot.intakeRotateClawServo.setPosition(0.446);
                break;
            case DEFAULT:
                setArmPosition(0.50);
                robot.intakeRotateArmServo.setPosition(0.385); // 0.385
                robot.intakeRotateClawServo.setPosition(0.446);
                break;
            case SCANNING:
                setArmPosition(0.62);
                robot.intakeRotateArmServo.setPosition(0.385); // 0.95
                break;
            case ROTATE:
                robot.intakeRotateArmServo.setPosition(0.95); // 0.95
                robot.intakeRotateClawServo.setPosition(clawPosition);
                break;
        }
    }
    public void updateSlideState(SlideState state) {
        if (slideState == state) {
            return;
        }
        this.slideState = state;
        switch (slideState) {
            case TRANSFERRING:
                robot.horizontalSlideActuator.setTargetPosition(0);
                break;
            case GRABBING_SAMPLE:
                robot.horizontalSlideActuator.setTargetPosition(175);
                break;
        }
    }

    private void setArmPosition(double pos) {
        robot.leftArmServo.setPosition(pos);
        robot.rightArmServo.setPosition(pos);
    }

    public String getLimelightResults() {
        return "tx: " + tx + " ty: " + ty + " pos: " + clawPosition;
    }
}
