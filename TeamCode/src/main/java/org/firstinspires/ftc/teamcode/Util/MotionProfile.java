package org.firstinspires.ftc.teamcode.Util;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class MotionProfile {
    public double initialPosition;
    public double finalPosition;
    public double distance;
    public double t1, t2, t3;
    public double totalTime;
    public double t1_stop_position;
    public double max_velocity;
    public double t2_stop_position;
    public boolean flipped = false;
    public double originalPos = 0;
    public double position = 0;

    public double velo, accel;

    public MotionProfile(double initialPosition, double finalPosition, double velo, double accel) {
        if (finalPosition < initialPosition) {
            flipped = true;
            this.originalPos = initialPosition;
            double temp = initialPosition;
            initialPosition = finalPosition;
            finalPosition = temp;
        }
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.distance = finalPosition - initialPosition;
        this.velo = velo;
        this.accel = accel;

        t1 = this.velo / this.accel;
        t3 = t1;
        t2 = Math.abs(distance) / this.velo - (t1 + t3) / 2;

        if (t2 < 0) {
            this.t2 = 0;

            double a = (this.accel / 2) * (1 - this.accel / -this.accel);
            double c = -distance;

            t1 = Math.sqrt(-4 * a * c) / (2 * a);
            t3 = -(this.accel * t1) / -this.accel;
            t1_stop_position = (this.accel * Math.pow(t1, 2)) / 2;

            max_velocity = this.accel * t1;

            t2_stop_position = t1_stop_position;
        } else {
            max_velocity = this.velo;
            t1_stop_position = (this.velo * t1) / 2;
            t2_stop_position = t1_stop_position + t2 * max_velocity;
        }

        totalTime = t1 + t2 + t3;
    }

    public double calculate(double time) {
        double velocity, acceleration, stage_time;
        if (time <= t1) {
            stage_time = time;
            acceleration = this.accel;
            velocity = acceleration * stage_time;
            position = velocity * stage_time / 2;
        } else if (time <= t1 + t2) {
            stage_time = time - t1;
            acceleration = 0;
            velocity = this.velo;
            position = t1_stop_position + stage_time * velocity;
        } else if (time <= totalTime) {
            stage_time = time - t1 - t2;
            acceleration = -this.accel;
            velocity = max_velocity - stage_time * this.accel;
            position = t2_stop_position + (max_velocity + velocity) / 2 * stage_time;
        } else {
            acceleration = 0;
            velocity = 0;
            position = finalPosition;
        }

        if (time <= totalTime) {
            if (flipped) {
                position = originalPos - position;
            } else {
                position = initialPosition + position;
            }
        } else {
            if (flipped) {
                position = initialPosition;
            } else {
                position = originalPos + position;
            }
        }

        return position;
    }
}
