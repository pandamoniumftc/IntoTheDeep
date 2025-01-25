package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.PandaMotor;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotorActuator;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotorEncoder;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Hardware.PandaServo;
import org.firstinspires.ftc.teamcode.Hardware.PandaServoActuator;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;

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
        RETRACT
    }
    public enum SlideState {
        TRANSFERRING,
        DEFAULT,
        SCANNING_SAMPLE,
        GRABBING_SAMPLE
    }
    public ClawState clawState;
    public ArmState armState;
    public SlideState slideState;
    public boolean adjusting = false;
    public Intake() {
        robot = PandaRobot.getInstance();

        robot.horizontalSlideActuator = new PandaMotorActuator(
                new PandaMotor[] {
                        robot.expansionHub.getMotor(0)
                                .setDirection(DcMotorSimple.Direction.REVERSE)
                                .setConfigurations(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                                .setCacheTolerance(0.1)
                },
                new PandaMotorEncoder(robot.expansionHub.getMotor(0).setDirection(DcMotorSimple.Direction.FORWARD))
        )
                .setPIDController(0.013, 0.00015, 0.0001)
                .setLimits(0.0, 180)
                .setMotionProfile(320, 480)
                .setTolerance(1)
                .setPowerThreshold(0.1);

        robot.intakeArmActuator = new PandaServoActuator(
                new PandaServo[] {
                        robot.expansionHub.getServo(0).setDirection(Servo.Direction.FORWARD),
                        robot.controlHub.getServo(0).setDirection(Servo.Direction.REVERSE)
                }
        ).setOffset(new double[] {0.0, 0.0});

        robot.intakeClawServo = robot.controlHub.getServo(2);
        robot.intakeRotateClawServo = robot.controlHub.getServo(3); // 0* = 0.445, 90* = 0.384, 180* = 0.327
        robot.intakeRotateArmServo = robot.controlHub.getServo(1);
        robot.intakeLightChain = robot.expansionHub.getServo(3);
    }

    @Override
    public void read() {
        robot.horizontalSlideActuator.read();
    }

    @Override
    public void loop() {
        robot.horizontalSlideActuator.loop();
        robot.intakeArmActuator.loop();
    }

    @Override
    public void write() {
        robot.horizontalSlideActuator.write();
        robot.intakeArmActuator.write();
    }

    public void updateClawState(ClawState state) {
        this.clawState = state;
        robot.intakeClawServo.setPosition((clawState == ClawState.OPENED) ? 0.68 : 0.54);
    }
    public void updateArmState(ArmState state) {
        this.armState = state;
        switch (armState) {
            case GRABBING:
                robot.intakeArmActuator.setPosition(0.765);
                robot.intakeRotateArmServo.setPosition(0.06);
                break;
            case TRANSFERRING:
                robot.intakeArmActuator.setPosition(0.46);
                robot.intakeRotateArmServo.setPosition(0.74);
                robot.intakeRotateClawServo.setPosition(0.445);
                break;
            case DEFAULT:
                robot.intakeArmActuator.setPosition(0.48);
                robot.intakeRotateArmServo.setPosition(0.06);
                robot.intakeRotateClawServo.setPosition(0.3835);
                break;
            case RETRACT:
                robot.intakeArmActuator.setPosition(0.48);
                robot.intakeRotateArmServo.setPosition(0.06);
                break;
        }
    }
    public void updateSlideState(SlideState state) {
        this.slideState = state;
        switch (slideState) {
            case TRANSFERRING:
                robot.horizontalSlideActuator.setTargetPosition(0);
                break;
            case DEFAULT:
                robot.horizontalSlideActuator.setTargetPosition(20);
                break;
            case SCANNING_SAMPLE:
                robot.horizontalSlideActuator.setTargetPosition(160);
                break;
            case GRABBING_SAMPLE:
                robot.horizontalSlideActuator.setTargetPosition(100);
                break;
        }
    }
}
