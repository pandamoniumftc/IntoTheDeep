package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.signum;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.Devices.Analog;
import org.firstinspires.ftc.teamcode.Devices.Motor;
import org.firstinspires.ftc.teamcode.Devices.Servo;
import org.firstinspires.ftc.teamcode.Robots.Globals;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.MotionProfile;

import java.io.IOException;

public class Intake extends AbstractSubsystem {
    HuaHua robot;
    public Motor slideM;
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
    final double InchesToRadians = 0.919327459151/8.0;
    double clawPos = 0.49;
    double slidePos = 2;
    private final double VMAX = 4.0, AMAX = 3.0;
    public Intake(AbstractRobot robot, int hsm, int se, int cs, int rs, int lws, int rws, int les, int res, int lE, int rE) {
        super(robot);
        this.robot = (HuaHua) robot;

        // init motor and servos
        slideM = this.robot.expansionHub.getMotor(
                abs(hsm),
                this.robot.expansionHub.getEncoder(abs(se), 751.8),
                new PID(0.3, 0.4, 0),
                new MotionProfile(0, 0, VMAX, AMAX),
                InchesToRadians
        );
        slideM.setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE);
        slideM.setVoltage(robot.controlHub.getVoltage());

        clawS = this.robot.expansionHub.getServo(cs);
        rotateS = this.robot.expansionHub.getServo(rs);
        lWristS = this.robot.expansionHub.getServo(lws);
        rWristS = this.robot.expansionHub.getServo(rws);
        lElbowS = this.robot.expansionHub.getServo(les);
        rElbowS = this.robot.expansionHub.getServo(res);
        //lEnc = this.robot.controlHub.getAnalog(lE);
        //rEnc = this.robot.controlHub.getAnalog(rE);

        lElbowS.addPresetPosition(0.52, 0); //+2
        rElbowS.addPresetPosition(0.62, 0); //-2
        lWristS.addPresetPosition(0.28, 0); //+6
        rWristS.addPresetPosition(0.69, 0); //-6

        lElbowS.addPresetPosition(0.48, 1); //46
        rElbowS.addPresetPosition(0.66, 1); //68
        lWristS.addPresetPosition(0.24, 1); //+4
        rWristS.addPresetPosition(0.73, 1); //-4

        lElbowS.addPresetPosition(0.21, 2); //-4
        rElbowS.addPresetPosition(0.91, 2); //+4
        lWristS.addPresetPosition(0.90, 2); //-4
        rWristS.addPresetPosition(0.07, 2); //+4

        clawState = ClawState.CLOSED;
        armState = ArmState.TRANSFERING;

    }

    @Override
    public void init() throws IOException {
        slideM.setInitialPosition();
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        slideM.setVoltage(robot.controlHub.getVoltage());
        if (armState == ArmState.TRANSFERING) {
            slideM.update();
        }
        else {

        }

        if (Globals.opMode == Globals.RobotOpMode.TELEOP) {
            robot.telemetry.addData("arm state", armState);
            robot.telemetry.addData("current : target", slideM.encoder.getRotation() + " " + (slideM.target + slideM.initial));
            robot.telemetry.addData("init : target", slideM.profile.initialPosition + " " + slideM.profile.finalPosition);
            robot.telemetry.addData("power", slideM.power);
            robot.telemetry.addData("slide", slidePos);
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
        clawPos = clip(clawPos, 0.43, 0.54);
        rotateS.setPosition(clawPos);
    }
    public void resetClawRotation() {
        rotateS.setPosition(0.43);
    }
    public void moveSlides(double jInput) {
        slidePos += jInput * 0.25;
        slidePos = clip(slidePos, 2, 8);
        slideM.setPosition(slidePos);
    }
}
