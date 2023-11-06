package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
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
import org.firstinspires.ftc.teamcode.sensors.RisingEdgeDetector;
@Config
@TeleOp(name = "LiftTest")
public class LiftTest extends CommandOpMode {
    public static final double liftLow = 140;
    public static final double liftHigh = 1720;
    public static final double armLeft = -360;
    public static final double armRight = 360;
    public static double vAdjust = 200;
    public static double aAdjust = 5000;
    public static double liftKp = 0.02;
    public static double liftKi = 0.01;
    public static double liftKgs = 0.15;
    public static double liftKgd = 0.0001;
    public static double armKp = 0.015;
    public static double armKi = 0.01;
    public static double liftVm = 3500;
    public static double liftAi = 30000;
    public static double liftAf = 20000;
    public static double armVm = 3000;
    public static double armAi = 30000;
    public static double armAf = 20000;
    public static double liftKa = 0.00001;
    public static double armKa = 0.000005;
    private DcMotor liftR;
    private DcMotor liftL;
    private PidfController liftPidf;
    private PidfController armPidf;
    private MotionProfile liftProfile;
    private MotionProfile armProfile;
    private boolean big;
    @Override
    public void initOpMode() {
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");
        liftL = hardwareMap.get(DcMotorEx.class,     "liftL");
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftPidf = new PidfController(new PidCoefficients(liftKp, liftKi, 0), x -> (x[0] > 80 ? liftKgs + x[0] * liftKgd : 0) + x[2] * liftKa);
        armPidf = new PidfController(new PidCoefficients(armKp, armKi, 0), x -> x[2] * armKa);
        liftProfile = new AsymProfile(new AsymConstraints(liftVm, liftAi, liftAf), 0, 0, 0, (liftLow + liftHigh) / 2, 0);
        armProfile = new DelayProfile(0, 0, 0, 0);
        big = false;
        Subsystem lift = (t, b) -> {
            liftPidf.set(liftProfile.getX(t));
            armPidf.set(armProfile.getX(t));
            double leftX = liftL.getCurrentPosition();
            double rightX = liftR.getCurrentPosition();
            liftPidf.update(t, leftX + rightX, liftProfile.getV(t), liftProfile.getA(t));
            armPidf.update(t,leftX - rightX, armProfile.getV(t), armProfile.getA(t));
            //liftL.setPower(liftPidf.get() + armPidf.get());
            //liftR.setPower(liftPidf.get() - armPidf.get());
            liftPidf.setConstants(new PidCoefficients(liftKp, liftKi, 0), x -> liftKgs + x[0] * liftKgd + x[2] * liftKa);
            armPidf.setConstants(new PidCoefficients(armKp, armKi, 0), x -> x[2] * armKa);
            big = gamepad1.right_trigger > 0.2;
            telemetry.addData("Lift Actual Position", leftX + rightX);
            telemetry.addData("Lift Estimated Position", liftProfile.getX(t));
            telemetry.addData("Arm Actual Position", leftX - rightX);
            telemetry.addData("Arm Estimated Position", armProfile.getX(t));
        };
        scheduler.register(lift);
        scheduler.schedule(FnCommand.repeat(d -> {
                if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                    scheduler.schedule(FnCommand.once(t -> {
                        double liftAdjust = liftAdjust(gamepad1.dpad_up, gamepad1.dpad_down);
                        double armAdjust = armAdjust(gamepad1.dpad_left, gamepad1.dpad_right);
                        if (liftAdjust != 0) {
                            liftProfile = SymProfile.extendSym(liftProfile, new SymConstraints(vAdjust, aAdjust),
                                    t, clip(liftProfile.getX(t) + liftAdjust, liftLow, liftHigh), 0);
                        }
                        if (armAdjust != 0) {
                            armProfile = SymProfile.extendSym(liftProfile, new SymConstraints(vAdjust, aAdjust),
                                    t, clip(armProfile.getX(t) + armAdjust, liftLow, liftHigh), 0);
                        }
                    }, lift));}}));
        scheduler.addListener(
                RisingEdgeDetector.listen(() -> gamepad1.a, new FnCommand(t ->
                        liftProfile = AsymProfile.extendAsym(liftProfile, new AsymConstraints(liftVm, liftAi, liftAf), t,
                        min(liftProfile.getX(t) + (liftHigh - liftLow) / (big ? 1 : 8), liftHigh), 0),
                        t -> {}, (t, b) -> {}, t -> t > max(liftProfile.getTf(), armProfile.getTf()), lift)),
                RisingEdgeDetector.listen(() -> gamepad1.b, new FnCommand(t ->
                        liftProfile = AsymProfile.extendAsym(liftProfile, new AsymConstraints(liftVm, liftAi, liftAf), t,
                        max(liftProfile.getX(t) - (liftHigh - liftLow) / (big ? 1 : 8), liftLow), 0),
                        t -> {}, (t, b) -> {}, t -> t > max(liftProfile.getTf(), armProfile.getTf()), lift)),
                RisingEdgeDetector.listen(() -> gamepad1.x, new FnCommand(t ->
                        armProfile = AsymProfile.extendAsym(armProfile, new AsymConstraints(armVm, armAi, armAf), t,
                        min(armProfile.getX(t) + (armRight - armLeft) / (big ? 1 : 8), armRight), 0),
                        t -> {}, (t, b) -> {}, t -> t > max(liftProfile.getTf(), armProfile.getTf()), lift)),
                RisingEdgeDetector.listen(() -> gamepad1.y, new FnCommand(t ->
                        armProfile = AsymProfile.extendAsym(armProfile, new AsymConstraints(armVm, armAi, armAf), t,
                        max(armProfile.getX(t) - (armRight - armLeft) / (big ? 1 : 8), armLeft), 0),
                        t -> {}, (t, b) -> {}, t -> t > max(liftProfile.getTf(), armProfile.getTf()), lift)));
    }
    private static double liftAdjust(boolean up, boolean down) {
        if (up) {
            return 25;
        } else if (down) {
            return -25;
        }
        return 0;
    }
    private static double armAdjust(boolean right, boolean left) {
        if (right) {
            return 10;
        } else if (left) {
            return -10;
        }
        return 0;
    }
}