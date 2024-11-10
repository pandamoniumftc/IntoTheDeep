package org.firstinspires.ftc.teamcode.Util.profile;

public class ProfileState {
    public double position, velocity, acceleration;
    public enum MotionState {
        NOT_STARTED,
        ACCELERATING,
        CRUISING,
        DECELERATING,
        DONE
    }
    public MotionState phase = MotionState.NOT_STARTED;
    public ProfileState(double x, double v, double a) {
        this.position = x;
        this.velocity = v;
        this.acceleration = a;
    }
    public ProfileState() {}
    public void updateState(double x, double v, double a) {
        this.position = x;
        this.velocity = v;
        this.acceleration = a;
    }
}
