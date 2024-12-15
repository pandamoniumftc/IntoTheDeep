package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.Motor;
import org.firstinspires.ftc.teamcode.Hardware.MotorActuator;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;

public class Outtake extends Subsystem {
    private Robot robot;
    public enum ArmState {
        TRANSFERING,
        SCORING
    }
    public ArmState armState;
    public enum ClawState {
        OPENED,
        CLOSED
    }
    public ClawState clawState;
    private final double VERTICAL_SLIDE_MM_TO_TICKS = 1;
    public Outtake() {
        robot = Robot.getInstance();

        robot.verticalSlidesActuator = new MotorActuator(
                new Motor[] {
                        robot.controlHub.getMotor(0)
                                .setDirection(Motor.Direction.FORWARD)
                                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE),
                        robot.controlHub.getMotor(0)
                                .setDirection(Motor.Direction.FORWARD)
                                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                },
                robot.controlHub.getEncoder(0)
        )
                .setPIDController(0.95, 0.4, 0.0)
                .setScale(VERTICAL_SLIDE_MM_TO_TICKS)
                .setMotionProfile(3.5, 3.5)
                .setTolerance(0.05);

        robot.outtakeClawServo = robot.controlHub.getServo(0);
        robot.outtakePivotServo = robot.controlHub.getServo(0);
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
        if (Globals.opMode == Globals.RobotOpMode.AUTO) {
            robot.verticalSlidesActuator.write();
        }
    }

    public void updateArmState(ArmState state) {
        this.armState = state;
        robot.outtakePivotServo.setPosition(armState == ArmState.TRANSFERING ? 0 : 0.85);
    }
    public void updateClawState(ClawState state) {
        this.clawState = state;
        robot.outtakeClawServo.setPosition(clawState == ClawState.CLOSED ? 0.65 : 1);
    }
    public void moveSlides(double jInput) {
        robot.verticalSlidesActuator.write(jInput);
    }
    public double getSlidesPosition() {
        return robot.verticalSlidesActuator.getPosition();
    }
}
