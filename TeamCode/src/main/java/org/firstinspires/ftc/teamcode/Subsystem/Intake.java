package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static com.qualcomm.robotcore.util.Range.scale;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.Encoder;
import org.firstinspires.ftc.teamcode.Hardware.Motor;
import org.firstinspires.ftc.teamcode.Hardware.MotorActuator;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Servo;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystem.CameraSystems.SampleAlignmentPipeline;

public class Intake extends Subsystem {
    private final Robot robot;
    public enum ClawState {
        OPENED,
        CLOSED
    }
    public enum ArmState {
        GRABBING,
        INTAKING,
        TRANSFERING,
        DEFAULT
    }
    public ClawState clawState;
    public ArmState armState;
    public double calculatedClawRotation;
    public Intake() {
        robot = Robot.getInstance();

        robot.horizontalSlideActuator = new MotorActuator(
                new Motor[] {
                        robot.expansionHub.getMotor(1)
                                .setDirection(Motor.Direction.FORWARD)
                                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                                .setDeadZone(0.1)
                },
                robot.expansionHub.getEncoder(1)
                        .setDirection(Encoder.Direction.FORWARD)
        )
                .setPIDController(0.01, 0.0, 0.0)
                .setLimits(0.0, 223.0)
                .setMotionProfile(500, 800)
                .setTolerance(5)
        ;

        robot.intakeClawServo = robot.controlHub.getServo(4);
        robot.intakeRotateServo = robot.controlHub.getServo(5);
        robot.intakeLeftWristServo = robot.controlHub.getServo(2);
        robot.intakeRightWristServo = robot.controlHub.getServo(3)
                .setDirection(Servo.Direction.REVERSE);
        robot.intakeLeftElbowServo = robot.controlHub.getServo(0);
        robot.intakeRightElbowServo = robot.controlHub.getServo(1)
                .setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void read() {
        robot.horizontalSlideActuator.read();
    }

    @Override
    public void loop() {
        robot.horizontalSlideActuator.loop();
    }

    @Override
    public void write() {
        robot.horizontalSlideActuator.write();
    }

    public void updateClawState(ClawState state) {
        this.clawState = state;
        robot.intakeClawServo.setPosition((clawState == ClawState.OPENED) ? 0.79 : 0.57);
    }
    public void updateArmState(ArmState state) {
        this.armState = state;
        switch (armState) {
            case GRABBING:
                setWristPosition(0.25);
                setElbowPosition(0.79);
                break;
            case INTAKING:
                setWristPosition(0.19);
                setElbowPosition(0.73);
                robot.intakeRotateServo.setPosition(0.385);
                break;
            case TRANSFERING:
                setWristPosition(0.87); //.87
                setElbowPosition(0.50); //.50
                robot.intakeRotateServo.setPosition(0.443);
                break;
            case DEFAULT:
                setWristPosition(0.87);
                setElbowPosition(0.54);
                robot.intakeRotateServo.setPosition(0.443);
                break;
        }
    }

    private void setWristPosition(double pos) {
        robot.intakeLeftWristServo.setPosition(pos);
        robot.intakeRightWristServo.setPosition(pos-0.1);
    }

    private void setElbowPosition(double pos) {
        robot.intakeLeftElbowServo.setPosition(pos);
        robot.intakeRightElbowServo.setPosition(pos+0.04);
    }
}
