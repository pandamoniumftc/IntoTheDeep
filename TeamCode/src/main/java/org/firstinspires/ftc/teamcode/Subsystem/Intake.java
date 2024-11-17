package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.signum;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import java.util.concurrent.TimeUnit;

public class Intake extends AbstractSubsystem {
    HuaHua robot;
    public Motor slideM;
    Servo clawS, rotateS, lWristS, rWristS, lElbowS, rElbowS;
    public enum ClawState {
        OPENED,
        CLOSED
    }
    public enum ArmState {
        GRABBING(0),
        INTAKING(1),
        TRANSFERING(2),
        DEFAULT(3);
        private int index;
        ArmState(int i) {
            this.index = i;
        }
    }
    public ClawState clawState;
    public ArmState armState;
    final double InchesToRadians = 1.5461/12.5;
    double clawPos = 0.49;
    private final double KP = 0.75, KI = 0.30, KD = 0, VMAX = 3.0, AMAX = 3.0;
    public Intake(AbstractRobot robot, int hsm, int se, int cs, int rs, int lws, int rws, int les, int res) {
        super(robot);
        this.robot = (HuaHua) robot;

        // init motor and servos
        slideM = this.robot.expansionHub.getMotor(
                hsm,
                this.robot.expansionHub.getEncoder(se, 751.8),
                new PID(KP, KI, KD),
                new MotionProfile(0, 0, VMAX, AMAX),
                InchesToRadians
        );
        slideM.setDirection(1);
        slideM.setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE);
        slideM.setVoltage(robot.controlHub.getVoltage());

        clawS = this.robot.expansionHub.getServo(cs);
        rotateS = this.robot.expansionHub.getServo(rs);
        lWristS = this.robot.expansionHub.getServo(lws);
        rWristS = this.robot.expansionHub.getServo(rws);
        lElbowS = this.robot.expansionHub.getServo(les);
        rElbowS = this.robot.expansionHub.getServo(res);

        lElbowS.addPresetPosition(0.52, 0); //+2
        rElbowS.addPresetPosition(0.62, 0); //-2
        lWristS.addPresetPosition(0.25, 0); //+6
        rWristS.addPresetPosition(0.72, 0); //-6

        lElbowS.addPresetPosition(0.48, 1); //46
        rElbowS.addPresetPosition(0.66, 1); //68
        lWristS.addPresetPosition(0.24, 1); //+4
        rWristS.addPresetPosition(0.73, 1); //-4

        lElbowS.addPresetPosition(0.21, 2); //-4
        rElbowS.addPresetPosition(0.91, 2); //+4
        lWristS.addPresetPosition(0.90, 2); //-4
        rWristS.addPresetPosition(0.07, 2); //+4

        lElbowS.addPresetPosition(0.25, 3); //-4
        rElbowS.addPresetPosition(0.87, 3); //+4
        lWristS.addPresetPosition(0.90, 3); //-4
        rWristS.addPresetPosition(0.07, 3); //+4

        clawState = ClawState.CLOSED;
        armState = ArmState.DEFAULT;
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
        robot.intake.slideM.setVoltage(robot.controlHub.getVoltage());
        robot.intake.slideM.update();
    }

    @Override
    public void stop() {

    }

    public void updateClawState(ClawState state) {
        this.clawState = state;
        clawS.setPosition((clawState == ClawState.OPENED) ? 0.79 : 0.57);
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
        clawPos += 1E-3 * (-lTrigger + rTrigger);
        clawPos = clip(clawPos, 0.42, 0.54);
        rotateS.setPosition(clawPos);
    }
    public void resetClawRotation() {
        rotateS.setPosition(0.42);
    }
}
