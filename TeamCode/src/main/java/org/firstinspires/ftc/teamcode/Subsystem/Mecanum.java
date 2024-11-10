package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sin;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.Devices.Motor;
import org.firstinspires.ftc.teamcode.Robots.Globals;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.Pose;

import java.io.IOException;

public class Mecanum extends AbstractSubsystem {
    HuaHua robot;
    public Motor frm, flm, brm, blm;
    public double[] power = new double[4];
    public Mecanum(AbstractRobot robot, int flm, int frm, int blm, int brm) {
        super(robot);
        this.robot = (HuaHua) robot;

        this.frm = this.robot.controlHub.getMotor(abs(frm));
        this.flm = this.robot.controlHub.getMotor(abs(flm));
        this.brm = this.robot.controlHub.getMotor(abs(brm));
        this.blm = this.robot.controlHub.getMotor(abs(blm));

        this.frm.setDirection((int) signum(frm));
        this.flm.setDirection((int) signum(flm));
        this.brm.setDirection((int) signum(brm));
        this.blm.setDirection((int) signum(blm));
    }

    @Override
    public void init() throws IOException {

    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        setDrivePower();

        //robot.telemetry.addData("angle", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        robot.telemetry.update();
    }

    @Override
    public void stop() {

    }

    public void setDrivePower() {
        flm.setPower(power[0]);
        frm.setPower(power[1]);
        blm.setPower(power[2]);
        brm.setPower(power[3]);
    }

    public void calculatePower(Pose p, double angle) {
        p.rotate(angle, new Pose(0, 0, 0));

        p.x = clip(p.x, -1, 1);
        p.y = clip(p.y, -1, 1);
        p.r = clip(p.r, -1, 1);

        power[0] = (-p.y+p.x+p.r); // flm
        power[1] = (-p.y-p.x-p.r); // frm
        power[2] = (-p.y-p.x+p.r); // blm
        power[3] = (-p.y+p.x-p.r); // brm

        double voltCorrection;
        if (Globals.opMode == Globals.RobotOpMode.AUTO) {
            voltCorrection = 12.0 / ((double) robot.controlHub.getVoltage() / 1000);
            for (int i = 0; i < power.length; i++) {
                power[i] = (abs(power[i]) < 0.01) ?
                        power[i] * voltCorrection :
                        (power[i] + signum(power[i]) * 0.085) * voltCorrection;
            }
        }

        double max = 1.0;
        for (double pow : power) max = Math.max(max, abs(pow));

        if (max > 1) {
            power[0] /= max;
            power[1] /= max;
            power[2] /= max;
            power[3] /= max;
        }
    }
}
