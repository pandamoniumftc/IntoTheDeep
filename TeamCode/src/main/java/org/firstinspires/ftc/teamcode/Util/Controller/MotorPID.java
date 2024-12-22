package org.firstinspires.ftc.teamcode.Util.Controller;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

public class MotorPID {
    double kp, ki, kd;
    public double pErr = 0.0, p = 0.0, i = 0.0, d = 0.0, iErr = 0.0, pD = 0.0, power = 0.0, prevPower = 0.0, alpha = 0.1;
    boolean reiniting = false, satCheck1 = false, satCheck2 = false;
    long stamp;

    public MotorPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        stamp = System.currentTimeMillis();
    }

    public double update(double state, double target) {
        p = target - state;

        if (reiniting) {
            reiniting = false;
            pErr = p;
        }

        double dt = (System.currentTimeMillis() - stamp) / 1E3;

        i += ki * p * dt;

        d = (p - pErr) / dt;
        d = alpha * d + (1 - alpha) * pD;
        pD = d;

        power = p * kp + i + d * kd;
        power = clip(power, -1, 1);

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

    public MotorPID setAlpha(double a) {
        alpha = a;
        return this;
    }
}
