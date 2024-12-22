package org.firstinspires.ftc.teamcode.Util;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class MotionProfile {
    public double aMax, vMax, distance, initial, target;
    public double acceleration_dt, deceleration_time, acceleration_distance, entire_dt, cruise_distance, cruise_dt;
    public MotionProfile(double referencePosition, double referenceTarget, double max_velocity, double max_acceleration) {
        this.initial = referencePosition;
        this.target = referenceTarget;
        this.distance = target - initial;
        if (max_velocity <= 0 || max_acceleration <= 0) {
            throw new IllegalArgumentException("Max velocity and acceleration must be greater than zero");
        }
        this.vMax = max_velocity;
        this.aMax = max_acceleration;

        acceleration_dt = vMax / aMax;
        acceleration_distance = 0.5 * aMax * Math.pow(acceleration_dt, 2);

        if (Math.abs(distance) < 2 * acceleration_distance) {
            // Adjust for triangular profile
            double adjustedAccelerationDistance = Math.abs(distance) / 2.0;
            acceleration_dt = Math.sqrt(2 * adjustedAccelerationDistance / aMax);
            acceleration_distance = adjustedAccelerationDistance;
            cruise_distance = 0;
            cruise_dt = 0;
        } else {
            // Full trapezoidal profile
            cruise_distance = Math.abs(distance) - 2 * acceleration_distance;
            cruise_dt = cruise_distance / vMax;
        }

        // calculate the time that we're at max velocity
        cruise_distance = abs(distance) - 2 * acceleration_distance;
        cruise_dt = cruise_distance / vMax;
        double deceleration_dt = acceleration_dt;
        deceleration_time = acceleration_dt + cruise_dt;

        entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
    }
    public double calculate(double time) {
        // if we're accelerating
        if (time < acceleration_dt) {
            return initial + signum(distance) * (0.5 * aMax * Math.pow(time, 2));
        }

        // if we're cruising
        else if (time < deceleration_time) {
            return initial + signum(distance) * (acceleration_distance + vMax * (time - acceleration_dt));
        }

        // if we're decelerating
        else if (time < entire_dt) {
            double d = time - deceleration_time;
            // use the kinematic equations to calculate the instantaneous desired position
            return initial + signum(distance) * (acceleration_distance + cruise_distance + vMax * d - 0.5 * aMax * Math.pow(d, 2));
        }

        // check if we're still in the motion profile
        else {
            return target;
        }
    }
}
