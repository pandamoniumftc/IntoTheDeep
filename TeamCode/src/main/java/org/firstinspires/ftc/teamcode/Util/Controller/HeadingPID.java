package org.firstinspires.ftc.teamcode.Util.Controller;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class HeadingPID {
    double kp, ki, kd;
    public double pErr = 0.0, p = 0.0, i = 0.0, d = 0.0;
    public Vector2d power = new Vector2d();
    boolean reiniting = false;
    ElapsedTime timer;
    public HeadingPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        timer = new ElapsedTime();
    }

    public Vector2d update(double state, double target) {
        p = target - state;

        if (reiniting) {
            reiniting = false;
            pErr = p;
        }

        double dt = timer.time(TimeUnit.SECONDS);

        i += ki * p * dt;

        d = (p - pErr) / dt;

        power = new Vector2d(p * kp + i + d * kd, 0).normalize();

        pErr = p;
        timer.reset();

        return power;
    }

    public void reInit() {
        if (!reiniting) {
            i = 0.0;
            timer.reset();
            reiniting = true;
        }
    }

    public boolean isFinished(double tolerance) {
        return abs(p) < tolerance;
    }
}
