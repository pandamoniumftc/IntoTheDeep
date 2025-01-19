package org.firstinspires.ftc.teamcode.Subsystem;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.PandaMotor;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotorActuator;
import org.firstinspires.ftc.teamcode.Hardware.PandaMotorEncoder;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
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
    public Outtake() {
        robot = PandaRobot.getInstance();

        robot.verticalSlidesActuator = new PandaMotorActuator(
                new PandaMotor[] {
                        robot.controlHub.getMotor(0)
                                .setDirection(DcMotorSimple.Direction.FORWARD)
                                .setConfigurations(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                                .setDeadZone(0.05),
                        robot.expansionHub.getMotor(3)
                                .setDirection(DcMotorSimple.Direction.REVERSE)
                                .setConfigurations(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                                .setDeadZone(0.05)
                },
                new PandaMotorEncoder(robot.expansionHub.getMotor(3))
        )
                .setPIDController(0.008, 0.0, 0.0)
                .setLimits(0, 4500)
                .setMotionProfile(6500, 6500)
                .setTolerance(100)
        ;

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
                robot.outtakePivotServo.setPosition(0.19); //0.19
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
        robot.outtakeClawServo.setPosition(clawState == ClawState.CLOSED ? 0.7 : 1.0);
    }
}
