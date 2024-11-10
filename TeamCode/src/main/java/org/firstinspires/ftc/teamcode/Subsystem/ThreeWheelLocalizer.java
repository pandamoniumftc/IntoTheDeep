package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractLocalizer;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.Devices.Encoder;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.Pose;

import java.util.Timer;

public class ThreeWheelLocalizer extends AbstractLocalizer {
    HuaHua robot;
    private Encoder rightEnc, leftEnc, frontEnc;
    private final double ENCODER_WHEEL_DIAMETER = 1.88; // diameter of the deadwheel
    private final int ENCODER_TICKS_PER_REVOLUTION = 2000; // ticks measured after one full revolution of the
    // deadwheel
    private final double ENCODER_WIDTH = 14.0; // distance between parallel deadwheels
    private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * ENCODER_WHEEL_DIAMETER;
    private Pose positionStamp;
    public ThreeWheelLocalizer(AbstractRobot robot, int right, int left, int front) {
        super(robot);
        this.robot = (HuaHua) robot;

        rightEnc = this.robot.controlHub.getEncoder(abs(right), ENCODER_TICKS_PER_REVOLUTION);
        leftEnc = this.robot.controlHub.getEncoder(abs(left), ENCODER_TICKS_PER_REVOLUTION);
        frontEnc = this.robot.controlHub.getEncoder(abs(front), ENCODER_TICKS_PER_REVOLUTION);

        rightEnc.setDirection((int) signum(right));
        leftEnc.setDirection((int) signum(left));
        frontEnc.setDirection((int) signum(front));

        encoder = new Pose(0, 0, 0);
        position = new Pose(0, 0, 0);
        positionStamp = new Pose(0, 0, 0);

        update();
    }

    @Override
    public void update() {
        encoder.setPose(new Pose(rightEnc.getEncoderCount(), leftEnc.getEncoderCount(), frontEnc.getEncoderCount()));

        position.setPose(combineSteps(position, convertTicksToDistance(new Pose(encoder.x - encoder.px,encoder.y - encoder.py,encoder.r - encoder.pr))));

        velocity = convertTicksToDistance(new Pose(rightEnc.getEncoderVelocity(), leftEnc.getEncoderVelocity(), frontEnc.getEncoderVelocity()));

        position.setPrevPose(position);
        encoder.setPrevPose(encoder);
    }

    @Override
    public void reset() {
        encoder.setPrevPose(new Pose(rightEnc.getEncoderCount(), leftEnc.getEncoderCount(), frontEnc.getEncoderCount()));
        position.setPose(new Pose(0,0,0));
        position.setPrevPose(new Pose(0,0,0));
    }

    public Pose convertTicksToDistance(Pose p) {
        // rotation -> distance
        double rightDist = p.x * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double leftDist = -p.y * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
        double dxR = -p.r * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;

        // forward motion
        double dyR = 0.5 * (rightDist + leftDist);

        // rotational motion
        double headingChangeRadians = (rightDist - leftDist) / ENCODER_WIDTH;

        return new Pose(dxR, dyR, headingChangeRadians);
    }

    public Pose combineSteps(Pose pos, Pose step) {
        double avgHeadingRadians = position.r + step.r / 2.0;

        double cos = cos(avgHeadingRadians);
        double sin = sin(avgHeadingRadians);
        return new Pose(pos.x + (step.x * sin + step.y * cos), pos.y + (-step.x * cos + step.y * sin), normalizeRadians(position.r + step.r));
    }
    public void stampPosition() {
        positionStamp = position;
    }
    public double calculateDisplacement() {
        return hypot(position.x - positionStamp.x, position.y - positionStamp.y);
    }
}
