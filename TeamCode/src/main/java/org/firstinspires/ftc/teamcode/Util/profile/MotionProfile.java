package org.firstinspires.ftc.teamcode.Util.profile;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

public class MotionProfile {
    double aMax, vMax, distance, initial, target;
    public ProfileState state;
    long timeStamp;
    public MotionProfile(double referenceTarget, ProfileConstraints constraints) {
        this.initial = 0;
        this.target = referenceTarget;
        this.distance = target;
        this.vMax = constraints.velocity;
        this.aMax = constraints.acceleration;

        state = new ProfileState();
        timeStamp = System.nanoTime();
    }
    public MotionProfile(double referencePosition, double referenceTarget, ProfileConstraints constraints) {
        this.initial = referencePosition;
        this.target = referenceTarget;
        this.distance = target - initial;
        this.vMax = constraints.velocity;
        this.aMax = constraints.acceleration;

        state = new ProfileState();
        timeStamp = System.nanoTime();
    }
    public double getTime() {
        return (System.nanoTime() - timeStamp) / 1E9;
    }
    public void resetTime() {
        timeStamp = System.nanoTime();
    }
    public boolean profileEnded() {
        return state.phase == ProfileState.MotionState.DONE;
    }
    public ProfileState getState() {
        return state;
    }
    public double getAccelMax() {
        return aMax;
    }
    public double getVeloMax() {
        return vMax;
    }
    public void update() {
        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = vMax / aMax;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = abs(distance) / 2;
        double acceleration_distance = 0.5 * aMax * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * aMax));
        }

        acceleration_distance = 0.5 * aMax * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        vMax = aMax * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = abs(distance) - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / vMax;
        double deceleration_time = acceleration_dt + cruise_dt;

        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;

        // if we're accelerating
        if (getTime() < acceleration_dt) {
            state.phase = ProfileState.MotionState.ACCELERATING;
            // use the kinematic equation for acceleration
            state.position = initial + signum(distance) * (0.5 * aMax * Math.pow(getTime(), 2));
            state.velocity = aMax * getTime();
            state.acceleration = aMax;
        }

        // if we're cruising
        else if (getTime() < deceleration_time) {
            state.phase = ProfileState.MotionState.CRUISING;
            acceleration_distance = 0.5 * aMax * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = getTime() - acceleration_dt;

            // use the kinematic equation for constant velocity
            state.position = initial + signum(distance) * (acceleration_distance + vMax * cruise_current_dt);
            state.velocity = vMax;
            state.acceleration = 0;
        }

        // if we're decelerating
        else if (getTime() < entire_dt) {
            state.phase = ProfileState.MotionState.DECELERATING;
            acceleration_distance = 0.5 * aMax * Math.pow(acceleration_dt, 2);
            cruise_distance = vMax * cruise_dt;
            deceleration_time = getTime() - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            state.position = initial + signum(distance) * (acceleration_distance + cruise_distance + vMax * deceleration_time - 0.5 * aMax * Math.pow(deceleration_time, 2));
            state.velocity = vMax - (aMax * deceleration_time);
            state.acceleration = -aMax;
        }

        // check if we're still in the motion profile
        else {
            state.phase = ProfileState.MotionState.DONE;
            state.position = target;
            state.velocity = 0;
            state.acceleration = 0;
        }
    }
}
