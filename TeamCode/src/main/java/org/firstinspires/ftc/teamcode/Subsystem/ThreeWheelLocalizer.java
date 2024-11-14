package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractLocalizer;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.Devices.Encoder;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.Pose;

public class ThreeWheelLocalizer extends AbstractLocalizer {
    HuaHua robot;
    private Encoder rightEnc, leftEnc, frontEnc;
    private final double ENCODER_WHEEL_DIAMETER = 1.88; // diameter of the deadwheel
    private final int ENCODER_TICKS_PER_REVOLUTION = 2000; // ticks measured after one full revolution of the
    // deadwheel
    private final double ENCODER_WIDTH = 14.0; // distance between parallel deadwheels
    private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * ENCODER_WHEEL_DIAMETER;
    public ThreeWheelLocalizer(AbstractRobot robot, int right, int left, int front) {
        super(robot);
        this.robot = (HuaHua) robot;

        rightEnc = this.robot.controlHub.getEncoder(right, ENCODER_TICKS_PER_REVOLUTION);
        leftEnc = this.robot.controlHub.getEncoder(left, ENCODER_TICKS_PER_REVOLUTION);
        frontEnc = this.robot.controlHub.getEncoder(front, ENCODER_TICKS_PER_REVOLUTION);

        encoder = new Pose(0, 0, 0);
        position = new Pose(0, 0, 0);
        prevEncoder = new Pose(0, 0, 0);
        prevPosition = new Pose(0, 0, 0);

        update();
    }

    @Override
    public void update() {
        encoder.setPose(new Pose(rightEnc.getCount(), leftEnc.getCount(), frontEnc.getCount()));

        position.setPose(combineSteps(position, convertTicksToDistance(new Pose(encoder.x - prevEncoder.x,encoder.y - prevEncoder.y,encoder.r - prevEncoder.r))));

        velocity = convertTicksToDistance(new Pose(rightEnc.getVelocity(), leftEnc.getVelocity(), frontEnc.getVelocity()));

        prevPosition.setPose(position);
        prevEncoder.setPose(encoder);
    }

    @Override
    public void reset() {
        prevEncoder.setPose(new Pose(rightEnc.getCount(), leftEnc.getCount(), frontEnc.getCount()));
        position.setPose(new Pose(0,0,0));
        prevPosition.setPose(new Pose(0,0,0));
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
}
