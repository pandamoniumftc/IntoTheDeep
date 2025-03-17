package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;
import org.firstinspires.ftc.teamcode.Util.Controller.PID;
import org.firstinspires.ftc.teamcode.Util.Pose2d;
import org.firstinspires.ftc.teamcode.Util.Spline;
import org.firstinspires.ftc.teamcode.Util.SplinePose;

import java.util.Arrays;

public class Mecanum extends Subsystem {
    private PandaRobot robot;
    private GamepadEx gamepad;
    TelemetryPacket packet;
    Canvas c;
    FtcDashboard dashboard;
    public double [] power = new double[4];
    public enum DriveState {
        DRIVE,
        BRAKE,
        ADJUST_TO_SAMPLE,
        GO_TO_POSITION,
        FOLLOWING_SPLINE,
        WAITING
    }
    public DriveState state;
    public Spline path;
    public Pose2d currentPosition = new Pose2d();
    public Pose2d currentVelocity = new Pose2d();
    public Pose2d targetPosition = new Pose2d();
    public final double MINIMUM_POWER_THRESHOLD = 0.2, POSITIONAL_THRESHOLD = 25.0, HEADING_THRESHOLD = 0.1, PIXEL_THRESHOLD = 5.0;
    public final double K_CENTRIPETAL = 1.0;
    public double xErr, yErr, hErr, MAX_POWER;
    public int index = 0, slowdownIndex = 3;
    private boolean slowDown = false;
    public double[] powers = new double[5];

    // TODO: X IS FORWARD/BACK and Y IS LEFT/RIGHT (+/-), IMPLEMENT SPLINE FOLLOWING ALGORITHM

    // USE FOR FOLLOWING SPLINES
    public PID splineTranslationalController = new PID(0.125, 0.0, 0.0);
    public PID splineHeadingController = new PID(6.0, 0.0, 0.0);

    // USE FOR MOVING TO POSITIONS
    public PID xController = new PID(0.014, 0.0, 0.002);
    public PID yController = new PID(0.014, 0.0, 0.002);
    public PID hController = new PID(3.0, 0.0, 0.002);

    // USE FOR ADJUSTING TO SAMPLES
    public PID xSampleController = new PID(0.030, 0.0, 0.00);//0.0085
    public PID ySampleController = new PID(0.050, 0.0, 0.00);//0.0060
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

        state = DriveState.BRAKE;

        packet = new TelemetryPacket();
        c = packet.fieldOverlay();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void read() {
        robot.odometry.update();
        currentPosition = robot.odometry.getPosition();
        currentVelocity = robot.odometry.getVelocity();
    }

    @Override
    public void loop() {
        if (path != null) {
            index = min(path.poses.size()-1, index);
            state = DriveState.FOLLOWING_SPLINE;
            MAX_POWER = path.poses.get(index).power;
            double lastRadius = path.poses.get(max(0,index-1)).getDistanceFromPoint(currentPosition);
            double radiusToPath = path.poses.get(index).getDistanceFromPoint(currentPosition);
            while (radiusToPath < 508.0 && index != path.poses.size()) {
                radiusToPath = path.poses.get(index).getDistanceFromPoint(currentPosition);
                if (lastRadius > radiusToPath && radiusToPath > 508.0/3.0){
                    break;
                }
                lastRadius = radiusToPath;
                index ++;
            }
            SplinePose pathTarget = path.poses.get(min(path.poses.size()-1,index));
            targetPosition = pathTarget.clone();
            /*if (path.poses.size() - 1 - index < slowdownIndex) {
                slowDown = true;
            }*/
            if (index == path.poses.size() && path.poses.get(path.poses.size()-1).getDistanceFromPoint(currentPosition) < 230.0){
                state = DriveState.GO_TO_POSITION;
                MAX_POWER/=2;
                path = null;
                index = 0;
                slowDown = false;
            }
            else {
                targetPosition.heading = Math.atan2(targetPosition.y - currentPosition.y, targetPosition.x - currentPosition.x);
            }
        }

        calculateErrors();

        switch(state) {
            case FOLLOWING_SPLINE:
                double radius = path.poses.get(min(path.poses.size()-1, index)).radius;
                powers[4] = radius;

                double drive = MAX_POWER * signum(xErr);

                if (slowDown) drive *= 0.5;

                double turn = splineHeadingController.update(hErr, 0.3);

                double centripetal = K_CENTRIPETAL * drive * drive / radius;

                double lastDist = currentPosition.getDistanceFromPoint(targetPosition);
                int i = Math.max(index-1,0);
                double dist = currentPosition.getDistanceFromPoint(path.poses.get(i));
                while (dist < lastDist && i > 0) {
                    i--;
                    lastDist = dist;
                    dist = currentPosition.getDistanceFromPoint(path.poses.get(i));
                }

                double strafe = yErr > 100.0 ? splineTranslationalController.update(yErr, 0.3) : 0;

                //c.strokeCircle(currentPosition.x / 25.4, currentPosition.y / 25.4, 9);
                //c.strokePolyline(path.xPoints, path.yPoints);

                //dashboard.sendTelemetryPacket(packet);

                powers[0] = drive;
                powers[1] = strafe;
                powers[2] = centripetal;
                powers[3] = turn;

                moveRobot(drive,strafe+centripetal,turn,false);
                break;
            case GO_TO_POSITION:
                PID();
                break;
            case ADJUST_TO_SAMPLE:
                adjustToSample();
                break;
            case BRAKE:
                stop();
                if (Globals.opMode == Globals.RobotOpMode.TELEOP) state = DriveState.DRIVE;
                else state = DriveState.WAITING;
                break;
            case WAITING:
                if (!reachedPosition()) {
                    resetIntegrals();
                    state = DriveState.GO_TO_POSITION;
                }
                break;
            case DRIVE:
                if (gamepad != null) moveRobot(gamepad);
                break;
        }
    }

    @Override
    public void write() {
        robot.frontLeftMotor.write(power[0]);
        robot.frontRightMotor.write(power[1]);
        robot.backLeftMotor.write(power[2]);
        robot.backRightMotor.write(power[3]);
    }

    public void test(GamepadEx g) {
        robot.frontLeftMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get() ? 1 : 0);
        robot.frontRightMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_UP).get() ? 1 : 0);
        robot.backLeftMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).get() ? 1 : 0);
        robot.backRightMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get() ? 1 : 0);
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
        moveRobot(g.getLeftY() * scale,-g.getLeftX() * 1.5 * scale,-g.getRightX() * scale, true);
    }

    public void calculateErrors() {
        if (state == DriveState.ADJUST_TO_SAMPLE) {
            xErr = -robot.intake.ty;
            yErr = robot.intake.tx;
        }
        else if (Globals.opMode == Globals.RobotOpMode.AUTO) {
            double x = targetPosition.x - currentPosition.x;
            double y = targetPosition.y - currentPosition.y;
            xErr = cos(currentPosition.heading)*x + sin(currentPosition.heading)*y;
            yErr = -sin(currentPosition.heading)*x + cos(currentPosition.heading)*y;
        }
        hErr = normalizeRadians(targetPosition.heading - currentPosition.heading);
    }

    public void PID() {
        double x = xController.update(xErr, MAX_POWER);
        double y = yController.update(yErr, MAX_POWER);
        double h = hController.update(hErr, MAX_POWER);
        moveRobot(x, y, h, false);
    }

    public void adjustToSample() {
        double x = abs(xErr) > PIXEL_THRESHOLD ? xSampleController.update(xErr, MAX_POWER) : 0;
        double y = abs(yErr) > PIXEL_THRESHOLD ? ySampleController.update(yErr, MAX_POWER) : 0;
        double h = hController.update(hErr, MAX_POWER);
        moveRobot(x, y, h, false);
    }

    public boolean reachedPosition() {
        double TRANSLATIONAL_THRESHOLD = (state == DriveState.ADJUST_TO_SAMPLE ? PIXEL_THRESHOLD : POSITIONAL_THRESHOLD);
        return Math.abs(xErr) < TRANSLATIONAL_THRESHOLD && Math.abs(yErr) < TRANSLATIONAL_THRESHOLD && Math.abs(hErr) < HEADING_THRESHOLD;
    }

    public boolean stalled() {
        return robot.frontLeftMotor.isStalled(MINIMUM_POWER_THRESHOLD) &&
                robot.frontRightMotor.isStalled(MINIMUM_POWER_THRESHOLD) &&
                robot.backLeftMotor.isStalled(MINIMUM_POWER_THRESHOLD) &&
                robot.backRightMotor.isStalled(MINIMUM_POWER_THRESHOLD);
    }

    public void resetIntegrals() {
        xController.reInit();
        yController.reInit();
        hController.reInit();
        xSampleController.reInit();
        ySampleController.reInit();
    }

    public void goToPosition(Pose2d pos, double maxPower) {
        this.MAX_POWER = maxPower;
        this.targetPosition = pos;
        resetIntegrals();
        state = DriveState.GO_TO_POSITION;
    }

    public void followSpline(Spline path) {
        this.path = path;
        resetIntegrals();
        state = DriveState.FOLLOWING_SPLINE;
    }

    public void startSampleAdjustment() {
        this.MAX_POWER = 0.3;
        resetIntegrals();
        targetPosition.heading = currentPosition.heading;
        state = DriveState.ADJUST_TO_SAMPLE;
    }

    public void brake() {
        state = DriveState.BRAKE;
    }

    public void setDriverGamepad(GamepadEx g) {
        this.gamepad = g;
        state = DriveState.DRIVE;
    }

    public void stop() {
        Arrays.fill(power, 0);
    }

    @SuppressLint("DefaultLocale")
    public String printMotorPowers() {
        return String.format("D: %.2f, S: %.2f, C: %.2f, T: %.2f, RADIUS: %.2f", powers[0], powers[1], powers[2], powers[3], powers[4]);
    }
}
