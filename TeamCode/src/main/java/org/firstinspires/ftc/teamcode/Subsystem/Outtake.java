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
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;

public class Outtake extends Subsystem {
    private PandaRobot robot;
    public enum ArmState {
        TRANSFERRING,
        CHECKING,
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
                                .setDirection(DcMotorSimple.Direction.REVERSE)
                                .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                                .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                                .setCacheTolerance(0.01)
                },
                Sensors.VERTICAL_SLIDES,
                true)
                .setPIDController(0.007, 0.0, 0.0) // 0.009
                .setLimits(0, 4000)
                .setMotionProfile(3900, 3900) // 2000, 3750
                .setTolerance(100)
                .setPowerThreshold(0.4);

        robot.outtakeClawServo = robot.controlHub.getServo(4);
        robot.outtakePivotServo = robot.expansionHub.getServo(1);

        robot.outtakeClawSensor = new AsyncRev2MSensor(hardwareMap.get(Rev2mDistanceSensor.class, "claw"));
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
            case TRANSFERRING:
                robot.outtakePivotServo.setPosition(0.15); //0.19
                break;
            case CHECKING:
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
        robot.outtakeClawServo.setPosition(clawState == ClawState.CLOSED ? 0.65 : 0.9); // TODO: REDUCE CLOSED CLAW POSITION GRIP
    }

    public void updateSlideState(SlideState state) {
        this.slideState = state;
        switch (state) {
            case DEFAULT:
                robot.verticalSlidesActuator.setPIDController(0.0012, 0.0, 0.0);
                robot.verticalSlidesActuator.setTargetPosition(0);
                break;
            case HIGH_BASKET:
                robot.verticalSlidesActuator.setPIDController(0.007, 0.0, 0.0);
                robot.verticalSlidesActuator.setTargetPosition(4000); // 3750 1012
                break;
            case LOW_BASKET:
                robot.verticalSlidesActuator.setPIDController(0.0012, 0.0, 0.0);
                robot.verticalSlidesActuator.setTargetPosition(1800); // 2200 594
                break;
            case HIGH_CHAMBER:
                robot.verticalSlidesActuator.setPIDController(0.002, 0.0, 0.0);
                robot.verticalSlidesActuator.setTargetPosition(2000); // 2000 540
                break;
            case LOW_CHAMBER:
                break;
            case LOWERED_FROM_HIGH:
                robot.verticalSlidesActuator.setPIDController(0.0007, 0.0, 0.0);
                robot.verticalSlidesActuator.setTargetPosition(1200); // 1400 378
                break;
            case GRABBED_SPECIMEN:
                robot.verticalSlidesActuator.setPIDController(0.001, 0.0, 0.0);
                robot.verticalSlidesActuator.setTargetPosition(500); // 300 81
                break;
        }
    }
}
