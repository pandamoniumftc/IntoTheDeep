package org.firstinspires.ftc.teamcode.Subsystem;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Motor;
import org.firstinspires.ftc.teamcode.Hardware.MotorActuator;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;

public class Outtake extends Subsystem {
    private Robot robot;
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
    public Outtake() {
        robot = Robot.getInstance();

        robot.verticalSlidesActuator = new MotorActuator(
                new Motor[] {
                        robot.controlHub.getMotor(1)
                                .setDirection(Motor.Direction.FORWARD)
                                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                                .setDeadZone(0.1),
                        robot.expansionHub.getMotor(0)
                                .setDirection(Motor.Direction.REVERSE)
                                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                                .setDeadZone(0.05)
                },
                robot.expansionHub.getEncoder(0).setDirection(Encoder.Direction.REVERSE)
        )
                .setPIDController(0.008, 0.0, 0.0)
                .setLimits(0, 4500)
                .setMotionProfile(6500, 6500)
                .setTolerance(100)
        ;

        robot.outtakeClawServo = robot.expansionHub.getServo(1);
        robot.outtakePivotServo = robot.expansionHub.getServo(2);
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

    public void updateArmState(ArmState state) {
        this.armState = state;
        switch (armState) {
            case TRANSFERING:
                robot.outtakePivotServo.setPosition(0.19);
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
        robot.outtakeClawServo.setPosition(clawState == ClawState.CLOSED ? 0.6 : 1);
    }
    public void moveSlides(double jInput) {
        robot.verticalSlidesActuator.write(jInput);
    }
}
