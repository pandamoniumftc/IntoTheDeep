package org.firstinspires.ftc.teamcode.Util.Controller;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.abs;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class DrivePID {
    double kp, ki, kd;
    public Vector2d pErr = new Vector2d(), p = new Vector2d(), i = new Vector2d(), d = new Vector2d(), power = new Vector2d();
    boolean reiniting = false;
    ElapsedTime timer;
    public DrivePID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        timer = new ElapsedTime();
    }

    public Vector2d update(Vector2d state, Vector2d target) {
        p = target.minus(state);

        if (reiniting) {
            reiniting = false;
            pErr = p;
        }

        double dt = timer.time(TimeUnit.NANOSECONDS) / 1E9;

        i.plus(p.scale(ki).scale(dt));

        d = p.minus(pErr).div(dt);

        power = p.scale(kp).plus(i).plus(d.scale(kd));

        pErr = p;
        timer.reset();

        return power;
    }

    public void reInit() {
        if (!reiniting) {
            i = new Vector2d();
            timer.reset();
            reiniting = true;
        }
    }

    public boolean isFinished(double tolerance) {
        return abs(p.magnitude()) < tolerance;
    }
}
