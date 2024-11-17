package org.firstinspires.ftc.teamcode.Subsystem;

import static com.qualcomm.robotcore.util.Range.clip;

import static org.firstinspires.ftc.teamcode.Robots.Globals.RobotOpMode.AUTO;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.Devices.Encoder;
import org.firstinspires.ftc.teamcode.Devices.Motor;
import org.firstinspires.ftc.teamcode.Devices.Servo;
import org.firstinspires.ftc.teamcode.Robots.Globals;
import org.firstinspires.ftc.teamcode.Robots.HuaHua;
import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.MotionProfile;

import java.io.IOException;

public class Outtake extends AbstractSubsystem {
    HuaHua robot;
    public Motor leftMotor, rightMotor;
    public Servo clawS, rotateS, lPivotS, rPivotS;
    final double ticksPerRev = 384.5;
    final double InchesToRadians = 2.099365;
    public enum ArmState {
        TRANSFERING(0),
        SCORING(1);
        private int index;
        ArmState(int i) {
            index = i;
        }
    }
    public ArmState armState;
    public enum ClawState {
        OPENED,
        CLOSED
    }
    public ClawState clawState;
    private final double KP = 0.05, KI = 0.0, KD = 0.0, VELO = 3.0, ACCEL = 2.0;
    public Outtake(AbstractRobot robot, int lsm, int rsm, int cs, int rs, int lps, int rps) {
        super(robot);
        this.robot = (HuaHua) robot;

        //profile = new MotionProfile(0, 0, new ProfileConstraints(1, 1));

        leftMotor = this.robot.expansionHub.getMotor(
                lsm,
                this.robot.expansionHub.getEncoder(rsm, ticksPerRev),
                new PID(KP, KI, KD),
                new MotionProfile(0, 0, VELO, ACCEL),
                InchesToRadians
        );
        rightMotor = this.robot.expansionHub.getMotor(
                rsm,
                this.robot.expansionHub.getEncoder(rsm, ticksPerRev),
                new PID(KP, KI, KD),
                new MotionProfile(0, 0, VELO, ACCEL),
                InchesToRadians
        );

        rightMotor.setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(1);
        leftMotor.setDirection(-1);
        rightMotor.encoder.setDirection(-1);

        clawS = this.robot.controlHub.getServo(cs);
        rotateS = this.robot.controlHub.getServo(rs);
        lPivotS = this.robot.controlHub.getServo(lps);
        rPivotS = this.robot.controlHub.getServo(rps);

        lPivotS.addPresetPosition(0, 0); //+2
        rPivotS.addPresetPosition(1, 0); //-2

        lPivotS.addPresetPosition(0.85, 1);
        rPivotS.addPresetPosition(0.15, 1);

        // rotate servo: transfer -> 1, score -> < 1
        rotateS.addPresetPosition(1, 0);
        rotateS.addPresetPosition(0.25, 1);

        armState = ArmState.TRANSFERING;
        clawState = ClawState.OPENED;
    }

    @Override
    public void init() throws IOException {
        leftMotor.setInitialPosition();
        rightMotor.setInitialPosition();
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        if (Globals.opMode == AUTO) {
            leftMotor.setVoltage(robot.controlHub.getVoltage());
            rightMotor.setVoltage(robot.controlHub.getVoltage());

            leftMotor.update();
            rightMotor.update();
        }

        if (Globals.telemetryEnable) {
            //robot.telemetry.addData("OUTTAKE ARM STATE", armState);
            //robot.telemetry.addData("OUTTAKE CLAW STATE", clawState);
            robot.telemetry.addData("OUTTAKE SLIDES POS", (15.5 + rightMotor.getPosition()) + " inches");
            robot.telemetry.addData("RIGHT", robot.outtake.rightMotor.power);
            robot.telemetry.addData("LEFT", robot.outtake.leftMotor.power);
        }
    }

    @Override
    public void stop() {

    }
    public void rotateClaw() {
        rotateS.setPresetPosition(armState.index);
    }
    public void moveArm() {
        lPivotS.setPresetPosition(armState.index);
        rPivotS.setPresetPosition(armState.index);
    }
    public void updateArmState(ArmState state) {
        this.armState = state;
    }
    public void updateClawState(ClawState state) {
        this.clawState = state;
        clawS.setPosition(state == ClawState.CLOSED ? 0.65 : 1);
    }
    public void moveSlides(double jInput) {
        leftMotor.setPower(jInput);
        rightMotor.setPower(jInput);
    }
    public double getSlidesPosition() {
        return rightMotor.getPosition();
    }
}
