package org.firstinspires.ftc.teamcode.control;
import java.util.function.ToDoubleFunction;

public class PidfController {
    double kp, ki, kd;
    ToDoubleFunction<double[]> kf;
    double setPoint = 0;
    double e = 0;
    double i = 0;
    double d = 0;
    double f = 0;
    double lastTime = 0;
    double lastE = 0;
    boolean first;
    public PidfController(double kp, double ki, double kd) {
        this(kp, ki, kd, x -> 0d);
    }
    public PidfController(double kp, double ki, double kd, ToDoubleFunction<double[]> kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        first = true;
    }
    public void reset() {
        i = 0;
    }
    public void setConstants(double kpNew, double kiNew, double kdNew, ToDoubleFunction<double[]> kf) {
        kp = kpNew;
        ki = kiNew;
        kd = kdNew;
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
        if (first) {
            first = false;
        } else {
            i += (e + lastE) * dt / 2;
        }
        d = (e - lastE) / dt;
        f = kf.applyAsDouble(x);
        if (lastE > 0 != e > 0) {
            reset();
        }
        lastTime = time;
        lastE = e;
    }
}