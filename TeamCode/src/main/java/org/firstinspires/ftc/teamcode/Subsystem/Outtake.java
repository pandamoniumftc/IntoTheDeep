package org.firstinspires.ftc.teamcode.Subsystem;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.AsyncRev2MSensor;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotor;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotorActuator;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;

public class Outtake {
    private PandaRobot robot;
    public enum ArmState {
        TRANSFERRING,
        SCORING_SAMPLE,
        SCORING_SPECIMEN
    }
    public ArmState armState;
    public enum ClawState {
        OPENED,
        CLOSED
    }
    public ClawState clawState;
    public enum SlideState {
        DEFAULT,
        HIGH_BASKET,
        HIGH_CHAMBER,
        SCORED_CHAMBER
    }
    public SlideState slideState;
    public Outtake(HardwareMap hardwareMap) {
        robot = PandaRobot.getInstance();

        robot.verticalSlidesActuator = new PandaMotorActuator(
                new PandaMotor[] {
                        robot.controlHub.getMotor(0)
                                .setDirection(DcMotorSimple.Direction.FORWARD)
                                .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                                .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                                .setCacheTolerance(0.01),
                        robot.expansionHub.getMotor(3)
                                .setDirection(DcMotorSimple.Direction.FORWARD)
                                .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                                .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                                .setCacheTolerance(0.01)
                },
                Sensors.VERTICAL_SLIDES,
                false)
                .setPIDController(0.01, 0.0, 0.0)
                .setLimits(0, 1200)
                .setTolerance(20);

        robot.outtakeClawServo = robot.controlHub.getServo(1);
        robot.outtakeElbowServo = robot.controlHub.getServo(4);

        robot.outtakeClawSensor = new AsyncRev2MSensor(hardwareMap.get(Rev2mDistanceSensor.class, "claw"));
    }

    public void update() {
        robot.verticalSlidesActuator.update();
    }

    public void write(double power) {
        robot.verticalSlidesActuator.write(power);
    }

    public void updateArmState(ArmState state) {
        this.armState = state;
        switch (armState) {
            case TRANSFERRING:
                robot.outtakeElbowServo.setPosition(0.57);
                break;
            case SCORING_SAMPLE:
                robot.outtakeElbowServo.setPosition(0.15);
                break;
            case SCORING_SPECIMEN:
                robot.outtakeElbowServo.setPosition(0);
                break;
        }
    }
    public void updateClawState(ClawState state) {
        this.clawState = state;
        robot.outtakeClawServo.setPosition(clawState == ClawState.OPENED ? 0.45 : 0.13);
    }

    public void updateSlideState(SlideState state) {
        this.slideState = state;
        switch (state) {
            case DEFAULT:
                robot.verticalSlidesActuator.setPIDController(0.006, 0, 0);
                robot.verticalSlidesActuator.setTargetPosition(0);
                break;
            case HIGH_BASKET:
                robot.verticalSlidesActuator.setPIDController(0.02, 0, 0);
                robot.verticalSlidesActuator.setTargetPosition(1200); // 3750 1012
                break;
            case HIGH_CHAMBER:
                robot.verticalSlidesActuator.setPIDController(0.02, 0, 0);
                robot.verticalSlidesActuator.setTargetPosition(600); // 2000 540
                break;
            case SCORED_CHAMBER:
                robot.verticalSlidesActuator.setPIDController(0.005, 0, 0);
                robot.verticalSlidesActuator.setTargetPosition(400); // 2000 540
                break;
        }
    }
}
