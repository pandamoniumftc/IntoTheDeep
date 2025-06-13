package org.firstinspires.ftc.teamcode.Util;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

public class PID {
    double kp, ki, kd;
    public double pErr = 0.0, p = 0.0, i = 0.0, d = 0.0, iErr = 0.0, pD = 0.0, power = 0.0, prevPower = 0.0, alpha = 0.1;
    boolean reiniting = false;
    long stamp;

    public PID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        stamp = System.currentTimeMillis();
    }

    public double update(double error, double maxPower) {
        p = error;

        if (reiniting) {
            reiniting = false;
            pErr = p;
        }

        double dt = (System.currentTimeMillis() - stamp) / 1E3;

        i += ki * p * dt;

        d = (p - pErr) / dt;

        power = p * kp + i + d * kd;
        power = clip(power, -maxPower, maxPower);

        pErr = p;
        stamp = System.currentTimeMillis();

        return power;
    }

    public void reInit() {
        if (!reiniting) {
            i=0.0;
            stamp = System.currentTimeMillis();
            reiniting = true;
        }
    }

    public boolean isFinished(double tolerance) {
        return abs(p) < tolerance;
    }
}
