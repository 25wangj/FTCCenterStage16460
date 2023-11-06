package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public static final double liftHigh = 1720;
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
    public static final SymConstraints adjustConstraints = new SymConstraints(100, 1000);
    private DcMotorEx liftR;
    private DcMotorEx liftL;
    private Servo claw;
    private PidfController liftPidf;
    private PidfController armPidf;
    private MotionProfile liftProfile;
    private MotionProfile armProfile;
    public Lift(LinearOpMode opMode, boolean auto) {
        liftR = opMode.hardwareMap.get(DcMotorEx.class, "liftR");
        liftL = opMode.hardwareMap.get(DcMotorEx.class, "liftL");
        claw = opMode.hardwareMap.get(Servo.class, "claw");
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        if (auto) {
            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        liftPidf = new PidfController(liftCoeffs, liftKf);
        armPidf = new PidfController(armCoeffs, armKf);
        liftProfile = new DelayProfile(0, 0, 0, 0);
        armProfile = new DelayProfile(0, 0, 0, 0);
    }
    public double restTime() {
        return max(liftProfile.getTf(), armProfile.getTf());
    }
    public Command adjust(double liftAdjust, double armAdjust) {
        return FnCommand.once(t -> {
            if (liftAdjust != 0) {
                liftProfile = SymProfile.extendSym(liftProfile, adjustConstraints, t,
                        clip(liftProfile.getX(t) + liftAdjust, liftLow, liftHigh), 0);
            }
            if (armAdjust != 0) {
                armProfile = SymProfile.extendSym(armProfile, adjustConstraints, t,
                        clip(armProfile.getX(t) + armAdjust, armLeft, armRight), 0);}}, this);
    }
    public Command goTo(double liftPos, double armPos) {
        return new FnCommand(t -> {
            liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints, t, liftPos, 0);
            armProfile = AsymProfile.extendAsym(armProfile, armConstraints, liftProfile.getTf(), armPos, 0);
        }, t -> {}, (t, b) -> {}, t -> t > restTime(), this);
    }
    public Command goBack() {
        return new FnCommand(t -> {
            armProfile = AsymProfile.extendAsym(armProfile, armConstraints, t, 0, 0);
            liftProfile = AsymProfile.extendAsym(liftProfile, liftConstraints, armProfile.getTf(), 0, 0);
        }, t -> {}, (t, b) -> {}, t -> t > restTime(), this);
    }
    public void setClaw(double pos) {
        claw.setPosition(pos);
    }
    @Override
    public void update(double time, boolean active) {
        liftPidf.set(liftProfile.getX(time));
        armPidf.set(armProfile.getX(time));
        double leftX = liftL.getCurrentPosition();
        double rightX = liftR.getCurrentPosition();
        liftPidf.update(time, leftX + rightX, liftProfile.getV(time), liftProfile.getA(time));
        armPidf.update(time,leftX - rightX, armProfile.getV(time), armProfile.getA(time));
        liftL.setPower(liftPidf.get() + armPidf.get());
        liftR.setPower(liftPidf.get() - armPidf.get());
    }
}