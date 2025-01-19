package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.max;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.PandaMotor;
import org.firstinspires.ftc.teamcode.Hardware.PandaRobot;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;
import org.opencv.core.Point;

public class Mecanum extends Subsystem {
    private PandaRobot robot;
    public double fl = 0.0, fr = 0.0, bl = 0.0, br = 0.0;
    public double tValue;
    public Vector2d t, h;
    public Point sample = new Point();
    public Mecanum() {
        robot = PandaRobot.getInstance();

        robot.frontLeftMotor = robot.expansionHub.getMotor(1)
                .setConfigurations(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(DcMotorSimple.Direction.FORWARD)
                .setDeadZone(0.01);

        robot.frontRightMotor = robot.expansionHub.getMotor(2)
                .setConfigurations(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setDeadZone(0.01);

        robot.backLeftMotor = robot.controlHub.getMotor(2)
                .setConfigurations(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(DcMotorSimple.Direction.FORWARD)
                .setDeadZone(0.01);

        robot.backRightMotor = robot.controlHub.getMotor(1)
                .setConfigurations(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(DcMotorSimple.Direction.REVERSE)
                .setDeadZone(0.01);
    }

    @Override
    public void read() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void write() {
        robot.frontLeftMotor.write(fl);
        robot.frontRightMotor.write(fr);
        robot.backLeftMotor.write(bl);
        robot.backRightMotor.write(br);
    }

    public void test(GamepadEx g) {
        robot.frontLeftMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get() ? 1 : 0);
        robot.frontRightMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_UP).get() ? 1 : 0);
        robot.backLeftMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).get() ? 1 : 0);
        robot.backRightMotor.write(g.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get() ? 1 : 0);
    }

    public void moveRobot(Vector2d left, Vector2d right, double angle) {
        t = left;
        h = right;
        double cosA = Math.cos(-angle);
        double sinA = Math.sin(-angle);
        double rx = left.getX() * cosA - left.getY() * sinA;
        double ry = left.getX() * sinA + left.getY() * cosA;

        double x = clip(rx, -1, 1);
        double y = clip(ry, -1, 1);
        double r = clip(right.getX(), -1, 1);

        double scale = min(1.0/(abs(x)+abs(y)+abs(r)),1.0);

        fl = (y+x+r) * scale; // flm
        fr = (y-x-r) * scale; // frm
        bl = (y-x+r) * scale; // blm
        br = (y+x-r) * scale; // brm
    }

    public void stopRobot() {
        moveRobot(new Vector2d(), new Vector2d(), 0);
    }

    public String print() {
        return fl + " " + fr + " " + bl + " " + br;
    }
}
