package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.max;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Motor;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.Subsystem;

public class Mecanum extends Subsystem {
    private Robot robot;
    public double[] power;
    public Mecanum() {
        robot = Robot.getInstance();

        robot.frontLeftMotor = robot.controlHub.getMotor(0)
                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(Motor.Direction.FORWARD);

        robot.frontRightMotor = robot.controlHub.getMotor(0)
                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(Motor.Direction.REVERSE);

        robot.backLeftMotor = robot.controlHub.getMotor(0)
                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(Motor.Direction.FORWARD);

        robot.backRightMotor = robot.controlHub.getMotor(0)
                .setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE)
                .setDirection(Motor.Direction.REVERSE);

        power = new double[4];
    }

    @Override
    public void read() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void write() {
        robot.frontLeftMotor.write(power[0]);
        robot.frontRightMotor.write(power[1]);
        robot.backLeftMotor.write(power[2]);
        robot.backRightMotor.write(power[3]);
    }

    public void moveRobot(Vector2d left, Vector2d right, double angle) {
        Vector2d l = left.normalize();
        l = l.rotateBy(-Math.toDegrees(angle));

        double x = clip(l.getX(), -1, 1);
        double y = clip(l.getY(), -1, 1);
        double r = clip(right.getX(), -1, 1);

        power[0] = (y+x+r); // flm
        power[1] = (y+x-r); // frm
        power[2] = (y-x+r); // blm
        power[3] = (y-x-r); // brm

        double max = 1.0;
        for (double pow : power) max = max(max, abs(pow));

        if (max > 1) {
            power[0] /= max;
            power[1] /= max;
            power[2] /= max;
            power[3] /= max;
        }
    }

    public void stopRobot() {
        moveRobot(new Vector2d(), new Vector2d(), 0);
    }
}
