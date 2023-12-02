package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.DelayProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.PidCoefficients;
import org.firstinspires.ftc.teamcode.control.PidfController;
import org.firstinspires.ftc.teamcode.control.SymConstraints;
import org.firstinspires.ftc.teamcode.control.SymProfile;
import java.util.function.ToDoubleFunction;
public class Lift implements Subsystem {
    public static final double liftLow = 140;
    public static final double liftHigh = 1710;
    public static final double armLeft = -360;
    public static final double armRight = 360;
    public static final double clawClosed = 0.7;
    public static final double clawOpen = 0.47;
    public static final PidCoefficients liftCoeffs = new PidCoefficients(0.02, 0.01, 0);
    public static final ToDoubleFunction<double[]> liftKf = x -> (x[0] > 80 ? 0.15 + 0.0001 * x[0] : -0.1) + 0.00001 * x[2];
    public static final PidCoefficients armCoeffs = new PidCoefficients(0.015, 0.01, 0);
    public static final ToDoubleFunction<double[]> armKf = x -> 0.000005 * x[2];
    public static final AsymConstraints liftConstraints = new AsymConstraints(3500, 30000, 20000);
    public static final AsymConstraints armConstraints = new AsymConstraints(3000, 30000, 20000);
    public static final SymConstraints adjustConstraints = new SymConstraints(500, 10000);
    private boolean rest = true;
    private DcMotorEx liftR;
    private DcMotorEx liftL;
    private Servo claw;
    private LynxModule hub;
    private PidfController liftPidf = new PidfController(liftCoeffs, liftKf);
    private PidfController armPidf = new PidfController(armCoeffs, armKf);
    private MotionProfile liftProfile = new DelayProfile(0, 0, 0, 0);
    private MotionProfile armProfile = new DelayProfile(0, 0, 0, 0);
    public Lift(LinearOpMode opMode, LynxModule hub, boolean auto) {
        liftR = opMode.hardwareMap.get(DcMotorEx.class, "liftR");
        liftL = opMode.hardwareMap.get(DcMotorEx.class, "liftL");
        claw = opMode.hardwareMap.get(Servo.class, "claw");
        this.hub = hub;
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        if (auto) {
            try {
                new LynxResetMotorEncoderCommand(hub, liftR.getPortNumber()).send();
                new LynxResetMotorEncoderCommand(hub, liftL.getPortNumber()).send();
            } catch (InterruptedException | LynxNackException e) {}
        }
    }
    public double restTime() {
        return max(liftProfile.tf(), armProfile.tf());
    }
    public Command adjust(double liftAdjust, double armAdjust) {
        return FnCommand.once(t -> {
            if (liftAdjust != 0) {
                liftProfile = SymProfile.extendSym(liftProfile, adjustConstraints, t,
                        clip(liftProfile.pos(t) + liftAdjust, liftLow, liftHigh), 0);
            }
            if (armAdjust != 0) {
                armProfile = SymProfile.extendSym(armProfile, adjustConstraints, t,
                        clip(armProfile.pos(t) + armAdjust, armLeft, armRight), 0);}}, this);
    }
    public Command goTo(double liftPos, double armPos, double delay) {
        return new FnCommand(t -> {
            if (rest) {
                rest = false;
                try {
                    new LynxResetMotorEncoderCommand(hub, liftR.getPortNumber()).send();
                    new LynxResetMotorEncoderCommand(hub, liftL.getPortNumber()).send();
                } catch (InterruptedException | LynxNackException e) {}
            }
            liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints, t, liftPos, 0);
            armProfile = AsymProfile.extendAsym(armProfile, armConstraints, liftProfile.tf() + delay, armPos, 0);
        }, t -> {}, (t, b) -> {}, t -> t > restTime(), this);
    }
    public Command goBack() {
        return new FnCommand(t -> {
            armProfile = AsymProfile.extendAsym(armProfile, armConstraints, t, 0, 0);
            liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints, armProfile.tf(), 0, 0);
        }, t -> {}, (t, b) -> rest = true, t -> t > restTime(), this);
    }
    public void setClaw(double pos) {
        claw.setPosition(pos);
    }
    @Override
    public void update(double time, boolean active) {
        if (rest) {
            liftL.setPower(-0.2);
            liftR.setPower(-0.2);
        } else {
            liftPidf.set(liftProfile.pos(time));
            armPidf.set(armProfile.pos(time));
            double leftX = liftL.getCurrentPosition();
            double rightX = liftR.getCurrentPosition();
            liftPidf.update(time, leftX + rightX, liftProfile.vel(time), liftProfile.accel(time));
            armPidf.update(time, leftX - rightX, armProfile.vel(time), armProfile.accel(time));
            liftL.setPower(liftPidf.get() + armPidf.get());
            liftR.setPower(liftPidf.get() - armPidf.get());
        }
    }
}