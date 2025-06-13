package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static com.qualcomm.robotcore.util.Range.scale;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotor;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotorActuator;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;

public class Intake {
    private final PandaRobot robot;

    public enum ClawState {
        OPENED,
        CLOSED
    }
    public enum ArmState {
        TRANSFERRING,
        SCANNING,
        GRABBING,
        RAISED
    }
    public enum SlideState {
        TRANSFERRING,
        GRABBING_SAMPLE,
        ADJUSTING
    }
    public ClawState clawState;
    public ArmState armState;
    public SlideState slideState;
    private boolean detected;
    private double clawPosition, tx, ty;
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
                .setPIDController(0.015, 0.0, 0.0)
                .setLimits(0.0, 190.0)
                .setMotionProfile(500, 1200)
                .setTolerance(5);

        robot.leftArmServo = robot.expansionHub.getServo(1).setDirection(Servo.Direction.FORWARD); // PREVIOUSLY EXPANSION 3
        robot.rightArmServo = robot.controlHub.getServo(3).setDirection(Servo.Direction.REVERSE); // PREVIOUSLY CONTROL 5
        robot.intakeClawServo = robot.controlHub.getServo(2); // PREVIOUSLY CONTROL 5
        robot.intakeRotateClawServo = robot.expansionHub.getServo(4); // 0* = 0.0, 45* = 0.15, 90* = 0.29, 135* = 0.44, 180* = 0.56
        robot.intakeRotateArmServo = robot.expansionHub.getServo(2);
        robot.intakeLightChain = robot.expansionHub.getServo(3); // PREVIOUSLY EXPANSION 5

        robot.limelight = hardwareMap.get(Limelight3A.class, "ll");
        robot.limelight.start();
        robot.limelight.pipelineSwitch(2);
    }

    public void update() {
        robot.horizontalSlideActuator.update();
        if (armState == ArmState.SCANNING) detected = updateLimeLightResults();
    }

    public void write(double power) {
        robot.horizontalSlideActuator.write(power);
    }

    public void updateClawState(ClawState state) {
        if (clawState == state) {
            return;
        }
        this.clawState = state;
        robot.intakeClawServo.setPosition((clawState == ClawState.OPENED) ? 0.45 : .3);
    }
    public void updateArmState(ArmState state) {
        if (armState == state) {
            return;
        }
        this.armState = state;
        switch (armState) {
            case GRABBING:
                setArmPosition(0.76);
                robot.intakeRotateArmServo.setPosition(0.94);
                robot.intakeRotateClawServo.setPosition(clawPosition);
                break;
            case TRANSFERRING:
                setArmPosition(0.46); // 0.28
                robot.intakeRotateArmServo.setPosition(0.38); // 0.94
                robot.intakeRotateClawServo.setPosition(0.84); // 0.84
                break;
            case SCANNING:
                setArmPosition(0.58);
                robot.intakeRotateArmServo.setPosition(0.38);
                robot.intakeRotateClawServo.setPosition(0.25);
                break;
            case RAISED:
                setArmPosition(0.64);
                robot.intakeRotateArmServo.setPosition(0.94);
                robot.intakeRotateClawServo.setPosition(clawPosition);
                break;
        }
    }
    public void updateSlideState(SlideState state) {
        if (slideState == state && state != SlideState.ADJUSTING) {
            return;
        }
        this.slideState = state;
        switch (slideState) {
            case TRANSFERRING:
                robot.horizontalSlideActuator.setTargetPosition(0);
                break;
            case GRABBING_SAMPLE:
                robot.horizontalSlideActuator.setTargetPosition(120);
                break;
            case ADJUSTING:
                robot.horizontalSlideActuator.setTargetPosition(robot.horizontalSlideActuator.getPosition() + getTy() * 1.75); // 1.75
                break;
        }
    }

    private void setArmPosition(double pos) {
        robot.leftArmServo.setPosition(pos);
        robot.rightArmServo.setPosition(pos);
    }

    public double scaleAngleToPos(double angle) {
        return scale(clip(map0to180ToSymmetric90(angle), -89, 89), -89, 89, 0, 0.54);
    }

    private double map0to180ToSymmetric90(double inputDegrees) {
        if (inputDegrees <= 90) {
            return inputDegrees;  // 0 to 90 maps directly
        } else {
            return inputDegrees - 180;  // 90 to 180 maps to decreasing values
        }
    }

    public boolean slideAdjusted() {
        return slideState == SlideState.ADJUSTING && robot.horizontalSlideActuator.isFinished();
    }

    public boolean updateLimeLightResults() {
        robot.limelight.updatePythonInputs(Globals.alliance == Globals.RobotAlliance.BLUE ? 0 : 1, Globals.targetingColorPiece ? 1 : 0, 0, 0, 0, 0, 0, 0);
        LLResult result = robot.limelight.getLatestResult();
        if (result.getTx() == 0 && result.getTy() == 0) {
            return false;
        }
        tx = result.getTx();
        ty = result.getTy();
        clawPosition = scaleAngleToPos(result.getPythonOutput()[0]);
        return true;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public String getLimelightResults() {
        return "tx: " + tx + " ty: " + ty + " pos: " + clawPosition;
    }

    public boolean objectDetected() {
        return detected;
    }
 }
