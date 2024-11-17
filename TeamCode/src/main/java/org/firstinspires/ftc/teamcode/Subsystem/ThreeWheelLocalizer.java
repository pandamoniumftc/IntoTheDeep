package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.Devices.Encoder;
import org.firstinspires.ftc.teamcode.Robots.Globals;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.Pose;

import java.io.IOException;

public class ThreeWheelLocalizer extends AbstractSubsystem {
    HuaHua robot;
    public Pose position, prevPosition, encoder, prevEncoder;
    private Encoder rightEnc, leftEnc, frontEnc;
    private final double ENCODER_WHEEL_DIAMETER = 1.889764; // diameter of the deadwheel
    private final int ENCODER_TICKS_PER_REVOLUTION = 2048; // ticks measured after one full revolution of the
    // deadwheel
    private final double WHEEL_CIRCUMFERENCE = ENCODER_WHEEL_DIAMETER * PI;
    private final double ENCODER_WIDTH = 8.625; // distance between parallel deadwheels
    public ThreeWheelLocalizer(AbstractRobot robot, int right, int left, int front) {
        super(robot);
        this.robot = (HuaHua) robot;

        rightEnc = this.robot.controlHub.getEncoder(right, ENCODER_TICKS_PER_REVOLUTION);
        leftEnc = this.robot.controlHub.getEncoder(left, ENCODER_TICKS_PER_REVOLUTION);
        frontEnc = this.robot.controlHub.getEncoder(front, ENCODER_TICKS_PER_REVOLUTION);

        rightEnc.setDirection(-1);
        leftEnc.setDirection(-1);
        frontEnc.setDirection(-1);

        encoder = new Pose(0, 0, 0);
        position = new Pose(0, 0, 0);
        prevEncoder = new Pose(leftEnc.getRotation(), rightEnc.getRotation(), frontEnc.getRotation());
        prevPosition = new Pose(0, 0, 0);

        update();
    }

    @Override
    public void init() throws IOException {
        reset();
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        update();

        if (Globals.telemetryEnable) {
            robot.telemetry.addData("POSITION", position.x + " " + position.y + " " + position.r);
            robot.telemetry.addData("ENCODER", encoder.x + " " + encoder.y + " " + encoder.r);
        }
    }

    @Override
    public void stop() {

    }

    public void update() {
        encoder.setPose(new Pose(leftEnc.getRotation(), rightEnc.getRotation(), frontEnc.getRotation()));

        Pose encStep = new Pose();
        encStep.setPose(encoder);

        encStep.applyOperation(prevEncoder, Pose.Operation.SUBTRACT);

        Pose newPos = blendSteps(position, calcMovementStep(encStep));

        position.x = newPos.x;
        position.y = newPos.y;
        position.r = newPos.r;

        prevPosition.setPose(position);
        prevEncoder.setPose(encoder);
    }

    public void reset() {
        prevEncoder.setPose(new Pose(leftEnc.getRotation(), rightEnc.getRotation(), frontEnc.getRotation()));
        position.setPose(new Pose(0,0,0));
        prevPosition.setPose(new Pose(0,0,0));
    }

    private Pose blendSteps(Pose cPos, Pose step) {
        Pose newPos = new Pose(cPos.x, cPos.y, cPos.r + step.r);

        double transDir = atan2(step.y, step.x);

        double d = step.magnitude();
        double a = step.r;
        if(a==0) a = 1E-6;
        double r = d / a;

        Pose arc = new Pose(cos(0.5 * PI - a) * r, sin(0.5 * PI - a) * r - r);

        arc.rotate(transDir + cPos.r);

        newPos.applyOperation(arc, Pose.Operation.ADD);

        return newPos;
    }

    private Pose calcMovementStep(Pose encStep) {
        Pose step = new Pose();

        encStep.applyOperation(ENCODER_WHEEL_DIAMETER/2.0, Pose.Operation.MULTIPLY);

        step.y = 0.5 * (encStep.x + encStep.y);

        step.r = (encStep.y - encStep.x) / (ENCODER_WIDTH);

        step.x = encStep.x;

        return step;
    }
}
