package org.firstinspires.ftc.teamcode.control;
import java.util.function.Function;
public abstract class PidfController {
    double kp, ki, kd;
    Function<double[], Double> kf;
    double setPoint = 0;
    double e = 0;
    double i = 0;
    double d = 0;
    double f = 0;
    double lastTime = 0;
    double lastE = 0;
    public PidfController(double kp, double ki, double kd) {
        this(kp, ki, kd, x -> 0d);
    }
    public PidfController(double kp, double ki, double kd, Function<double[], Double> kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }
    public void reset() {
        i = 0;
    }
    public void setConstants(double kpNew, double kiNew, double kdNew) {
        kp = kpNew;
        ki = kiNew;
        kd = kdNew;
    }
    public void set(double newSetPoint) {
        setPoint = newSetPoint;
    }
    public double get() {
        return kp * e + ki * i + kd * d + f;
    }
    public void update(double time, double... x) {
        double dt = time - lastTime;
        e = setPoint - x[0];
        i += (e + lastE) * dt / 2;
        d = (e - lastE) / dt;
        f = kf.apply(x);
        if (lastE > 0 != e > 0) {
            reset();
        }
        lastTime = time;
        lastE = e;
    }
}