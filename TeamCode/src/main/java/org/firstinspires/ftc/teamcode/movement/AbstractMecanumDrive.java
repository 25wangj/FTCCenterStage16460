package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.PidCoefficients;
import org.firstinspires.ftc.teamcode.control.PidfController;
public abstract class AbstractMecanumDrive extends Drivetrain {
    protected DcMotorEx fr;
    protected DcMotorEx fl;
    protected DcMotorEx br;
    protected DcMotorEx bl;
    private double trackWidth;
    private double ks;
    private double kv;
    private double ka;
    private double strafeMult;
    private PidfController xPid;
    private PidfController yPid;
    private PidfController turnPid;
    public AbstractMecanumDrive(double trackWidth, double ks, double kv, double ka, double strafeMult, PidCoefficients moveCoeffs, PidCoefficients turnCoeffs, AsymConstraints moveConstraints, AsymConstraints turnConstraints, boolean auto) {
        super(auto, moveConstraints, turnConstraints);
        this.trackWidth = trackWidth;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.strafeMult = strafeMult;
        xPid = new PidfController(moveCoeffs);
        yPid = new PidfController(moveCoeffs);
        turnPid = new PidfController(turnCoeffs);
    }
    public void setPowers(double x, double y, double t) {
        double d = max(abs(x) + abs(strafeMult * y) + abs(t), 1);
        fr.setPower(offset((x + strafeMult * y + t) / d, ks));
        fl.setPower(offset((x - strafeMult * y - t) / d, ks));
        br.setPower(offset((x - strafeMult * y + t) / d, ks));
        bl.setPower(offset((x + strafeMult * y - t) / d, ks));
    }
    @Override
    public void follow(double time) {
        Pose setPos = traj.pos(time);
        Pose setVel = traj.vel(time);
        Pose acPos = pose();
        Pose acVel = vel();
        Vec locPosError = acPos.vec().combo(1, setPos.vec(), -1).rotate(-acPos.h);
        Vec locVelError = acVel.vec().combo(1, setVel.vec(), -1).rotate(-acPos.h);
        Vec locVel = setVel.vec().rotate(-acPos.h);
        Vec locAccel = traj.accel(time).rotate(-acPos.h);
        xPid.derivUpdate(time, locVelError.x, locPosError.x);
        yPid.derivUpdate(time, locVelError.y, locPosError.y);
        turnPid.derivUpdate(time, acVel.h - setVel.h, ((acPos.h - setPos.h) % (2 * PI) + 3 * PI) % (2 * PI) - PI);
        System.out.println("Turn Power" + turnPid.get());
        setPowers(xPid.get() + kv * locVel.x + ka * locAccel.x, strafeMult * (yPid.get() + kv * locVel.y + ka * locAccel.y),
                turnPid.get() + kv * setVel.h * trackWidth);
    }
    private double offset(double a, double b) {
        return a + signum(a) * b;
    }
}
