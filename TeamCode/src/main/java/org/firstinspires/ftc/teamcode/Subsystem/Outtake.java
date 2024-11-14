package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.Devices.Encoder;
import org.firstinspires.ftc.teamcode.Devices.Motor;
import org.firstinspires.ftc.teamcode.Devices.Servo;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.MotionProfile;

import java.io.IOException;

public class Outtake extends AbstractSubsystem {
    HuaHua robot;
    public Motor leftMotor, rightMotor;
    Servo clawS, rotateS, lPivotS, rPivotS;
    PID controller;
    MotionProfile profile;
    Encoder encoder;
    final int ticksPerRev = 4096;
    final double ticksToDistance = 2.5;
    public enum ArmState {
        TRANSFERING(0),
        SCORING(1);
        private int index;
        ArmState(int i) {
            index = i;
        }
    }
    public ArmState armState;
    public enum ClawState {
        OPENED,
        CLOSED
    }
    public ClawState clawState;
    double slidePos = 0;
    public Outtake(AbstractRobot robot, int lsm, int rsm, int cs, int rs, int lps, int rps) {
        super(robot);
        this.robot = (HuaHua) robot;

        encoder = this.robot.controlHub.getEncoder(Math.abs(lsm), ticksPerRev);
        controller = new PID(1, 0, 0);
        //profile = new MotionProfile(0, 0, new ProfileConstraints(1, 1));

        leftMotor = this.robot.controlHub.getMotor(Math.abs(lsm), encoder, controller, profile, ticksToDistance);
        rightMotor = this.robot.controlHub.getMotor(Math.abs(rsm), encoder, controller, profile, ticksToDistance);

        clawS = this.robot.controlHub.getServo(cs);
        rotateS = this.robot.controlHub.getServo(rs);
        lPivotS = this.robot.controlHub.getServo(lps);
        rPivotS = this.robot.controlHub.getServo(rps);

        armState = ArmState.TRANSFERING;
        clawState = ClawState.OPENED;
    }

    @Override
    public void init() throws IOException {
        leftMotor.setInitialPosition();
        rightMotor.setInitialPosition();
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        leftMotor.update();
        rightMotor.update();
    }

    @Override
    public void stop() {

    }
    public void rotateClaw() {
        rotateS.setPresetPosition(armState.index);
    }
    public void moveArm() {
        lPivotS.setPresetPosition(armState.index);
        rPivotS.setPresetPosition(armState.index);
    }
    public void updateArmState(ArmState state) {
        this.armState = state;
    }
    public void updateClawState(ClawState state) {
        this.clawState = state;
        clawS.setPosition(state == ClawState.CLOSED ? 1 : 0);
    }
    public void moveSlides(double jInput) {
        slidePos += 1 * (jInput);
        slidePos = clip(slidePos, 5, 60);
        leftMotor.setPosition(slidePos);
        rightMotor.setPosition(slidePos);
    }
}
