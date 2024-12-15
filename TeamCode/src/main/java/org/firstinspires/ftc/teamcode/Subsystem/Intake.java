package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static com.qualcomm.robotcore.util.Range.scale;

import com.qualcomm.robotcore.hardware.DcMotor;

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
    private final double HORIZONTAL_SLIDE_MM_TO_TICKS = (0.58777633289987/255.0) * 384.5;
    public Intake() {
        robot = Robot.getInstance();

        robot.horizontalSlideActuator = new MotorActuator(
                new Motor[] {
                        robot.controlHub.getMotor(0)
                                .setDirection(Motor.Direction.FORWARD)
                                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                },
                robot.controlHub.getEncoder(0)
                        .setDirection(Encoder.Direction.REVERSE)
        )
                .setPIDController(2.0/1000.0, 8.0/10000.0, 0)
                .setScale(HORIZONTAL_SLIDE_MM_TO_TICKS)
                .setLimits(35, 250)
                .setMotionProfile(127.0, 127.0)
                .setTolerance(10);

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
        if (armState == ArmState.INTAKING) {
            calculatedClawRotation = getRotatedClawPosition(robot.sampleAlignmentPipeline.getSampleAngle());
        }
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
        // wrist: -.66 -> +.01
        // elbow: +.27 -> +.04
        switch (armState) {
            case GRABBING:
                setWristPosition(0.24);
                setElbowPosition(0.79);
                break;
            case INTAKING:
                setWristPosition(0.17);
                setElbowPosition(0.73);
                robot.intakeRotateServo.setPosition(0.385);
                break;
            case TRANSFERING:
                setWristPosition(0.85);
                setElbowPosition(0.48);
                robot.intakeRotateServo.setPosition(0.3285);
                break;
            case DEFAULT:
                setWristPosition(0.86);
                setElbowPosition(0.54);
                robot.intakeRotateServo.setPosition(0.3285);
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

    public double getRotatedClawPosition(double sampleAngle) {
        return scale(sampleAngle, 0, 180, 0.4415, 0.3285);
    }
}
