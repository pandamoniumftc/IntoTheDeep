package org.firstinspires.ftc.teamcode.Subsystem;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.PandaMotor;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotorActuator;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;

public class Outtake extends Subsystem {
    private PandaRobot robot;
    public enum ArmState {
        TRANSFERING,
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
        LOW_BASKET,
        HIGH_CHAMBER,
        LOW_CHAMBER,
        LOWERED_FROM_HIGH,
        GRABBED_SPECIMEN
    }
    public SlideState slideState;
    public Outtake() {
        robot = PandaRobot.getInstance();

        robot.verticalSlidesActuator = new PandaMotorActuator(
                new PandaMotor[] {
                        robot.controlHub.getMotor(0)
                                .setDirection(DcMotorSimple.Direction.FORWARD)
                                .setConfigurations(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                                .setCacheTolerance(0.01),
                        robot.expansionHub.getMotor(3)
                                .setDirection(DcMotorSimple.Direction.REVERSE)
                                .setConfigurations(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                                .setCacheTolerance(0.01)
                },
                Sensors.VERTICAL_SLIDES,
                true)
                .setPIDController(0.009, 0.0, 0.0) // 0.009
                .setLimits(0, 4500)
                .setMotionProfile(2000, 3750) // 2000, 3750
                .setTolerance(100)
                .setPowerThreshold(0.4);

        robot.outtakeClawServo = robot.controlHub.getServo(4);
        robot.outtakePivotServo = robot.expansionHub.getServo(1);
    }

    @Override
    public void read() {
        robot.verticalSlidesActuator.read();
    }

    @Override
    public void loop() {
        robot.verticalSlidesActuator.loop();
    }

    @Override
    public void write() {
        robot.verticalSlidesActuator.write();
    }

    public void write(double power) {
        robot.verticalSlidesActuator.write(power);
    }

    public void updateArmState(ArmState state) {
        this.armState = state;
        switch (armState) {
            case TRANSFERING:
                robot.outtakePivotServo.setPosition(0.15); //0.19
                break;
            case SCORING_SAMPLE:
                robot.outtakePivotServo.setPosition(0.65);
                break;
            case SCORING_SPECIMEN:
                robot.outtakePivotServo.setPosition(0.85);
                break;
        }
    }
    public void updateClawState(ClawState state) {
        this.clawState = state;
        robot.outtakeClawServo.setPosition(clawState == ClawState.CLOSED ? 0.63 : 1.0);
    }

    public void updateSlideState(SlideState state) {
        this.slideState = state;
        switch (state) {
            case DEFAULT:
                robot.verticalSlidesActuator.setTargetPosition(0);
                break;
            case HIGH_BASKET:
                robot.verticalSlidesActuator.setTargetPosition(3750);
                break;
            case LOW_BASKET:
                robot.verticalSlidesActuator.setTargetPosition(2200);
                break;
            case HIGH_CHAMBER:
                robot.verticalSlidesActuator.setTargetPosition(2000);
                break;
            case LOW_CHAMBER:
                break;
            case LOWERED_FROM_HIGH:
                robot.verticalSlidesActuator.setTargetPosition(1400);
                break;
            case GRABBED_SPECIMEN:
                robot.verticalSlidesActuator.setTargetPosition(300);
                break;
        }
    }
}
