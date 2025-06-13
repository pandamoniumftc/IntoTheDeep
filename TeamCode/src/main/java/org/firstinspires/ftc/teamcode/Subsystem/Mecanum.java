package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.max;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Util.MathFunctions;
import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.Pose2d;
import org.firstinspires.ftc.teamcode.Util.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;

public class Mecanum {
    private PandaRobot robot;
    private GamepadEx gamepad;
    public double [] power = new double[4];
    public enum DriveState {
        DRIVING,
        BRAKING,
        ADJUSTING_TO_SAMPLE,
        MOVING_TO_TARGET_POSITION,
        FOLLOWING_PURE_PURSUIT_PATH,
        WAITING
    }

    public DriveState state;
    public ArrayList<Waypoint> path;
    public Pose2d currentPosition = new Pose2d();
    public Pose2d currentVelocity = new Pose2d();
    public Pose2d targetPosition = new Pose2d();
    public Waypoint recentWaypoint;
    public Vector2d driveVector = new Vector2d();
    public final double POSITIONAL_THRESHOLD = 25.0, HEADING_THRESHOLD = 0.05, PIXEL_TO_MM = 4.5076;
    public double xErr, yErr, hErr, MAX_POWER;
    private boolean headingAssist = false;

    // TODO: X IS FORWARD/BACK and Y IS LEFT/RIGHT (+/-), IMPLEMENT SPLINE FOLLOWING ALGORITHM

    // USE FOR MOVING TO POSITIONS
    public PID xController = new PID(0.012, 0.0, 0.001);
    public PID yController = new PID(0.012, 0.0, 0.001);
    public PID hController = new PID(2.0, 0.0, 0.001);

    // USE FOR FINAL ADJUSTMENT
    public PID adjustXController = new PID(0.02, 0.0, 0.001);
    public PID adjustYController = new PID(0.02, 0.0, 0.001);
    public PID adjustHController = new PID(3.0, 0.0, 0.001);
    public Mecanum(HardwareMap hardwareMap) {
        robot = PandaRobot.getInstance();

        robot.frontLeftMotor = robot.expansionHub.getMotor(1)
                .setDirection(DcMotorSimple.Direction.FORWARD)
                .setCacheTolerance(0.01);

        robot.frontRightMotor = robot.expansionHub.getMotor(2)
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setCacheTolerance(0.01);

        robot.backLeftMotor = robot.controlHub.getMotor(3)
                .setDirection(DcMotorSimple.Direction.FORWARD)
                .setCacheTolerance(0.01);

        robot.backRightMotor = robot.controlHub.getMotor(1)
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setCacheTolerance(0.01);

        robot.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        robot.odometry.setOffsets(-55, 110); // -55, 110
        robot.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        robot.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD); // REVERSED, FORWARD

        state = DriveState.BRAKING;
    }

    public Mecanum(HardwareMap hardwareMap, GamepadEx gamepad) {
        robot = PandaRobot.getInstance();

        robot.frontLeftMotor = robot.expansionHub.getMotor(1)
                .setDirection(DcMotorSimple.Direction.FORWARD)
                .setCacheTolerance(0.01);

        robot.frontRightMotor = robot.expansionHub.getMotor(2)
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setCacheTolerance(0.01);

        robot.backLeftMotor = robot.controlHub.getMotor(3)
                .setDirection(DcMotorSimple.Direction.FORWARD)
                .setCacheTolerance(0.01);

        robot.backRightMotor = robot.controlHub.getMotor(1)
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setCacheTolerance(0.01);

        robot.odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        setDriverGamepad(gamepad);

        robot.odometry.setOffsets(-55, 60); // -55, 110
        robot.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        robot.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD); // REVERSED, FORWARD

        state = DriveState.BRAKING;
    }

    public void update() {
        robot.odometry.update();
        currentPosition = robot.odometry.getPosition();
        currentVelocity = robot.odometry.getVelocity();

        if (path != null) {
            recentWaypoint = getFollowPointPath(path, recentWaypoint.followDistance);
            MAX_POWER = recentWaypoint.moveSpeed;
            targetPosition = recentWaypoint.position;
        }

        calculateErrors();

        switch(state) {
            case FOLLOWING_PURE_PURSUIT_PATH:
                PP();
                break;
            case MOVING_TO_TARGET_POSITION:
                PID();
                break;
            case ADJUSTING_TO_SAMPLE:
                adjustPID();
                break;
            case BRAKING:
                stop();
                if (Globals.opMode == Globals.RobotOpMode.TELEOP) state = DriveState.DRIVING;
                else state = DriveState.WAITING;
                break;
            case WAITING:
                if (!reachedPosition()) {
                    resetIntegrals();
                    state = DriveState.MOVING_TO_TARGET_POSITION;
                }
                break;
            case DRIVING:
                if (gamepad != null) moveRobot(gamepad);
                break;
        }

        robot.frontLeftMotor.write(power[0]);
        robot.frontRightMotor.write(power[1]);
        robot.backLeftMotor.write(power[2]);
        robot.backRightMotor.write(power[3]);
    }

    public void test(GamepadEx g) {
        //robot.frontLeftMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get() ? 1 : 0);
        //robot.frontRightMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_UP).get() ? 1 : 0);
        //robot.backLeftMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).get() ? 1 : 0);
        //robot.backRightMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get() ? 1 : 0);
    }

    /**
     * @param forward controls forward (+) and backward (-) movement
     * @param side controls left (+) and right (-) movement
     * @param turn controls CCW (+) and CW (-) rotations
     * @param globally if the inputted power should be applied based on the robot's heading (in terms of global coordinate system)
     */
    public void moveRobot(double forward, double side, double turn, boolean globally) {
        double drive = globally ? forward * Math.cos(-currentPosition.heading) - side * sin(-currentPosition.heading) : forward;
        double strafe = globally ? forward * sin(-currentPosition.heading) + side * Math.cos(-currentPosition.heading) : side;

        double scale = min(1.0/(abs(drive)+abs(strafe)+abs(turn)),1.0);

        // CHANGED STRAFE AND TURN SIGNS (NEGATED)
        power[0] = (drive-strafe-turn) * scale; // flm
        power[1] = (drive+strafe+turn) * scale; // frm
        power[2] = (drive+strafe-turn) * scale; // blm
        power[3] = (drive-strafe+turn) * scale; // brm
    }

    public void moveRobot(GamepadEx g) {
        double scale = 0.7 * (1.0 - g.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) + 0.3;
        moveRobot(g.getLeftY() * scale,-g.getLeftX() * 1.5 * scale, headingAssist ? hController.update(hErr, MAX_POWER) : -g.getRightX() * scale, true);
    }

    public void calculateErrors() {
        xErr = targetPosition.x - currentPosition.x;
        yErr = targetPosition.y - currentPosition.y;
        hErr = normalizeRadians(targetPosition.heading - currentPosition.heading);
    }

    public void PP() {
        double distance = Math.hypot(xErr, yErr);
        double normX = xErr / distance;
        double normY = yErr / distance;

        double movementXPower = clip(normX, -MAX_POWER, MAX_POWER);
        double movementYPower = clip(normY, -MAX_POWER, MAX_POWER);
        double movementHPower = hController.update(hErr, MAX_POWER);

        driveVector = new Vector2d(movementXPower, movementYPower);

        moveRobot(movementXPower, movementYPower, movementHPower, true);

        if (path.get(path.size()-1).position.getDistanceFromPoint(robot.drive.currentPosition) < 250) {
            targetPosition = path.get(path.size()-1).position;
            MAX_POWER = path.get(path.size()-1).moveSpeed;
            path = null;
            resetIntegrals();
            state = DriveState.MOVING_TO_TARGET_POSITION;
        }
    }

    public void PID() {
        double x = xController.update(xErr, MAX_POWER);
        double y = yController.update(yErr, MAX_POWER);
        double h = hController.update(hErr, MAX_POWER);

        driveVector = new Vector2d(x, y);

        moveRobot(x, y, h, true);
    }

    public void adjustPID() {
        double x = adjustXController.update(xErr, MAX_POWER);
        double y = adjustYController.update(yErr, MAX_POWER);
        double h = adjustHController.update(hErr, MAX_POWER);
        moveRobot(x, y, h, true);
    }

    public boolean reachedPosition() {
        return Math.abs(xErr) < POSITIONAL_THRESHOLD && Math.abs(yErr) < POSITIONAL_THRESHOLD && Math.abs(hErr) < HEADING_THRESHOLD;
    }

    public void resetIntegrals() {
        xController.reInit();
        yController.reInit();
        hController.reInit();
        adjustXController.reInit();
        adjustYController.reInit();
        adjustHController.reInit();
    }

    public void goToPosition(Pose2d pos, double maxPower) {
        this.MAX_POWER = maxPower;
        this.targetPosition = pos;
        resetIntegrals();
        state = DriveState.MOVING_TO_TARGET_POSITION;
    }

    public void followPath(ArrayList<Waypoint> path) {
        this.path = path;
        recentWaypoint = path.get(0);
        MAX_POWER = recentWaypoint.moveSpeed;
        resetIntegrals();
        state = DriveState.FOLLOWING_PURE_PURSUIT_PATH;
    }

    public void startSampleAdjustment() {
        this.MAX_POWER = 0.6;
        resetIntegrals();
        robot.intake.updateSlideState(Intake.SlideState.ADJUSTING);
        Vector2d localError = new Vector2d(0, robot.intake.getTx() * PIXEL_TO_MM);
        localError = localError.rotateBy(Math.toDegrees(robot.drive.currentPosition.heading));
        robot.drive.targetPosition = new Pose2d(currentPosition.x + localError.getX(), currentPosition.y + localError.getY(), robot.drive.currentPosition.heading);
        state = DriveState.ADJUSTING_TO_SAMPLE;
    }

    public void enableTargetAngle(int degrees) {
        this.MAX_POWER = 0.7;
        headingAssist = true;
        resetIntegrals();
        robot.drive.targetPosition.heading = Math.toRadians(degrees);
    }

    public void disableTargetAngle() {
        headingAssist = false;
    }

    public void brake() {
        state = DriveState.BRAKING;
    }

    public void setDriverGamepad(GamepadEx g) {
        this.gamepad = g;
        state = DriveState.DRIVING;
    }

    public void stop() {
        Arrays.fill(power, 0);
    }
    public Waypoint getFollowPointPath(ArrayList<Waypoint> path, double lookaheadRadius) {
        Waypoint follow = new Waypoint(path.get(0));

        // Generate all line segments and find intersections
        for (int i = 0; i < path.size() - 1; i++) {
            Waypoint p1 = path.get(i);
            Waypoint p2 = path.get(i + 1);
            ArrayList<Pose2d> intersections = MathFunctions.lineCircleIntersections(p1.position, p2.position, currentPosition, lookaheadRadius);

            double closestAngle = Double.MAX_VALUE;

            for (Pose2d intersection : intersections) {
                double angle = Math.atan2(intersection.y - currentPosition.y, intersection.x - currentPosition.x);
                double deltaAngle = Math.abs(normalizeRadians(angle - driveVector.angle()));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    follow = p1;
                    follow.setPoint(intersection);
                }
            }
        }
        return follow;
    }

    @SuppressLint("DefaultLocale")
    public String printMotorPowers() {
        return String.format("FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", power[0], power[1], power[2], power[3]);
    }
}
