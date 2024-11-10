package org.firstinspires.ftc.teamcode.Util.Controllers;

public class PID {
    double kp, ki, kd;
    double initial;
    double pErr = 0.0, p = 0.0, i = 0.0, d = 0.0, power = 0.0, pTime;
    boolean reiniting = false;

    public PID(double kp, double ki, double kd, double initial) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.initial = initial;

        pTime = System.nanoTime();
    }

    public double update(double state, double target) {
        p = target - state;

        if (reiniting) {
            reiniting = false;
            pErr = p;
        }

        double t = System.nanoTime();
        double dt = (t - pTime) / (1E9);

        i += ki * p * dt;

        d = (p - pErr) / dt;

        power = p * kp + i + d * kd;
        power = Math.max(Math.min(power, 1), -1);

        pErr = p;
        pTime = t;

        return power;
    }

    public void reInit() {
        i=0.0;
        reiniting = true;
        pTime = System.nanoTime();
    }
}
