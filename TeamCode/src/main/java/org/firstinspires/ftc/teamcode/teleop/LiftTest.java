package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Listener;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.DelayProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.PidfController;
@Config
@TeleOp(name = "LiftTest")
public class LiftTest extends CommandOpMode {
    public static final double liftLow = 140;
    public static final double liftHigh = 1740;
    public static final double armLeft = -360;
    public static final double armRight = 360;
    public static double liftKp = 0.02;
    public static double liftKi = 0.01;
    public static double liftKgs = 0.2;
    public static double liftKgd = 0.0001;
    public static double armKp = 0.015;
    public static double armKi = 0.01;
    public static double liftVm = 3000;
    public static double liftAi = 30000;
    public static double liftAf = 20000;
    public static double armVm = 2500;
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
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftPidf = new PidfController(liftKp, liftKi, 0, x -> liftKgs + x[0] * liftKgd + x[2] * liftKa);
        armPidf = new PidfController(armKp, armKi, 0, x -> x[2] * armKa);
        liftProfile = new AsymProfile(liftVm, liftAi, liftAf, 0, 0, 0, (liftLow + liftHigh) / 2, 0);
        armProfile = new DelayProfile(0, 0, 0, 0);
        big = false;
        Subsystem lift = (t, b) -> {
            liftPidf.set(liftProfile.getX(t));
            armPidf.set(armProfile.getX(t));
            double leftX = liftL.getCurrentPosition();
            double rightX = liftR.getCurrentPosition();
            liftPidf.update(t, leftX + rightX, liftProfile.getV(t), liftProfile.getA(t));
            armPidf.update(t,leftX - rightX, armProfile.getV(t), armProfile.getA(t));
            liftL.setPower(liftPidf.get() + armPidf.get());
            liftR.setPower(liftPidf.get() - armPidf.get());
            liftPidf.setConstants(liftKp, liftKi, 0, x -> liftKgs + x[0] * liftKgd + x[2] * liftKa);
            armPidf.setConstants(armKp, armKi, 0, x -> x[2] * armKa);
            big = gamepad1.right_trigger > 0.2;
            telemetry.addData("Lift Actual Position", leftX + rightX);
            telemetry.addData("Lift Estimated Position", liftProfile.getX(t));
            telemetry.addData("Arm Actual Position", leftX - rightX);
            telemetry.addData("Arm Estimated Position", armProfile.getX(t));
        };
        scheduler.register(lift);
        scheduler.addListener(
                Listener.risingEdge(() -> gamepad1.a, new FnCommand(t ->
                        liftProfile = AsymProfile.extendAsym(liftProfile, liftVm, liftAi, liftAf, t,
                        min(liftProfile.getX(t) + (liftHigh - liftLow) / (big ? 1 : 8), liftHigh), 0),
                        t -> {}, (t, b) -> {}, t -> t > max(liftProfile.getTf(), armProfile.getTf()), lift)),
                Listener.risingEdge(() -> gamepad1.b, new FnCommand(t ->
                        liftProfile = AsymProfile.extendAsym(liftProfile, liftVm, liftAi, liftAf, t,
                        max(liftProfile.getX(t) - (liftHigh - liftLow) / (big ? 1 : 8), liftLow), 0),
                        t -> {}, (t, b) -> {}, t -> t > max(liftProfile.getTf(), armProfile.getTf()), lift)),
                Listener.risingEdge(() -> gamepad1.x, new FnCommand(t ->
                        armProfile = AsymProfile.extendAsym(armProfile, armVm, armAi, armAf, t,
                        min(armProfile.getX(t) + (armRight - armLeft) / (big ? 1 : 8), armRight), 0),
                        t -> {}, (t, b) -> {}, t -> t > max(liftProfile.getTf(), armProfile.getTf()), lift)),
                Listener.risingEdge(() -> gamepad1.y, new FnCommand(t ->
                        armProfile = AsymProfile.extendAsym(armProfile, armVm, armAi, armAf, t,
                        max(armProfile.getX(t) - (armRight - armLeft) / (big ? 1 : 8), armLeft), 0),
                        t -> {}, (t, b) -> {}, t -> t > max(liftProfile.getTf(), armProfile.getTf()), lift)));
    }
}
