package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.signum;
import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.Devices.Analog;
import org.firstinspires.ftc.teamcode.Devices.Encoder;
import org.firstinspires.ftc.teamcode.Devices.Motor;
import org.firstinspires.ftc.teamcode.Devices.Servo;
import org.firstinspires.ftc.teamcode.Robots.Globals;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.Controllers.PID;
import org.firstinspires.ftc.teamcode.Util.profile.MotionProfile;

import java.io.IOException;

public class Intake extends AbstractSubsystem {
    HuaHua robot;
    public Motor hSlideM;
    Encoder slideEnc;
    PID controller;
    MotionProfile profile;
    Servo clawS, rotateS, lWristS, rWristS, lElbowS, rElbowS;
    Analog lEnc, rEnc;
    double elbowAngle;
    public enum ClawState {
        OPENED,
        CLOSED
    }
    public enum ArmState {

        GRABBING(0),
        INTAKING(1),
        TRANSFERING(2);
        private int index;
        ArmState(int i) {
            this.index = i;
        }
    }

    public ClawState clawState;
    public ArmState armState;
    final double ticksToDistance = 2.5;
    double clawPos = 0.49;
    int slidePos = 0;
    public Intake(AbstractRobot robot, int hsm, int se, int cs, int rs, int lws, int rws, int les, int res, int lE, int rE) {
        super(robot);
        this.robot = (HuaHua) robot;

        // init motor, servos, and enc
        //slideEnc = this.robot.controlHub.getEncoder(se, 4096);
        //controller = new PID(1, 0, 0, 0);
        //profile = new MotionProfile(0, 0, new ProfileConstraints(1, 1, 1));
        //hSlideM = this.robot.controlHub.getMotor(abs(hsm), slideEnc, controller, profile, ticksToDistance);

        clawS = this.robot.controlHub.getServo(cs);
        rotateS = this.robot.controlHub.getServo(rs);
        lWristS = this.robot.controlHub.getServo(lws);
        rWristS = this.robot.controlHub.getServo(rws);
        lElbowS = this.robot.controlHub.getServo(les);
        rElbowS = this.robot.controlHub.getServo(res);
        //lEnc = this.robot.controlHub.getAnalog(lE);
        //rEnc = this.robot.controlHub.getAnalog(rE);

        //hSlideM.setDirection((int) signum(hsm));

        lElbowS.addPresetPosition(0.48, 0);
        rElbowS.addPresetPosition(0.66, 0);
        lElbowS.addPresetPosition(0.46, 1); // -3
        rElbowS.addPresetPosition(0.68, 1);
        lElbowS.addPresetPosition(0.25, 2); // -21
        rElbowS.addPresetPosition(0.87, 2);
        lWristS.addPresetPosition(0.22, 0);
        rWristS.addPresetPosition(0.75, 0);
        lWristS.addPresetPosition(0.20, 1);
        rWristS.addPresetPosition(0.77, 1);
        lWristS.addPresetPosition(0.94, 2);
        rWristS.addPresetPosition(0.03, 2);

        clawState = ClawState.CLOSED;
        armState = ArmState.TRANSFERING;
    }

    @Override
    public void init() throws IOException {
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        //hSlideM.update();
        if (Globals.opMode == Globals.RobotOpMode.TELEOP) {
            robot.telemetry.addData("arm state", armState);
            robot.telemetry.addData("arm state", armState);
            robot.telemetry.addData("arm state", armState);
            robot.telemetry.update();
        }
    }

    @Override
    public void stop() {

    }

    public void updateClawState(ClawState state) {
        this.clawState = state;
        clawS.setPosition((clawState == ClawState.OPENED) ? 0.74 : 0.57);
    }
    public void moveWrist() {
        lWristS.setPresetPosition(armState.index);
        rWristS.setPresetPosition(armState.index);
    }
    public void moveElbow() {
        lElbowS.setPresetPosition(armState.index);
        rElbowS.setPresetPosition(armState.index);
    }
    public void updateArmState(ArmState state) {
        this.armState = state;
    }
    public void rotateClaw(double lTrigger, double rTrigger) {
        clawPos += 5E-4 * (-lTrigger + rTrigger);
        clawPos = clip(clawPos, 0.42, 0.55);
        rotateS.setPosition(clawPos);
    }
    public void resetClawRotation() {
        rotateS.setPosition(0.42);
    }
    public void moveSlides(double jInput) {
        slidePos += 1 * (jInput);
        slidePos = clip(slidePos, 0, 35);
        hSlideM.setPosition(slidePos);
    }
}
