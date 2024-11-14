package org.firstinspires.ftc.teamcode.Util;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class PID {
    double kp, ki, kd;
    public double pErr = 0.0, p = 0.0, i = 0.0, d = 0.0, power = 0.0;
    boolean reiniting = false;
    long stamp;

    public PID(double kp, double ki, double kd) {
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

        power = p * kp + i + d * kd;
        power = clip(power, -1, 1);

        pErr = p;
        stamp = System.currentTimeMillis();

        return power;
    }
    public double update(double state, double stateVelocity, double target) {
        p = target - state;

        if (reiniting) {
            reiniting = false;
        }

        double dt = (System.currentTimeMillis() - stamp) / 1E3;

        i += ki * p * dt;

        d = stateVelocity;

        power = p * kp + i + d * kd;
        power = clip(power, -1, 1);

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
}
