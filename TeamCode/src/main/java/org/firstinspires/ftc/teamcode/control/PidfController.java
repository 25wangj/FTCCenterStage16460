package org.firstinspires.ftc.teamcode.control;
import java.util.function.ToDoubleFunction;
public class PidfController {
    private PidCoefficients coeffs;
    ToDoubleFunction<double[]> kf;
    private double setPoint = 0;
    private double e = 0;
    private double i = 0;
    private double d = 0;
    private double f = 0;
    private double lastTime = Double.NaN;
    private double lastE = 0;
    public PidfController(PidCoefficients coeffs) {
        this(coeffs, x -> 0d);
    }
    public PidfController(PidCoefficients coeffs, ToDoubleFunction<double[]> kf) {
        this.coeffs = coeffs;
        this.kf = kf;
    }
    public void reset() {
        i = 0;
    }
    public void setConstants(PidCoefficients coeffs, ToDoubleFunction<double[]> kf) {
        this.coeffs = coeffs;
        this.kf = kf;
    }
    public void setConstants(PidCoefficients coeffs) {
        this.coeffs = coeffs;
    }
    public void set(double newSetPoint) {
        setPoint = newSetPoint;
    }
    public double get() {
        return coeffs.kp * e + coeffs.ki * i + coeffs.kd * d + f;
    }
    public void update(double time, double... x) {
        e = setPoint - x[0];
        if (!Double.isNaN(lastTime)) {
            double dt = time - lastTime;
            i += (e + lastE) * dt / 2;
            d = (e - lastE) / dt;
        }
        f = kf.applyAsDouble(x);
        lastTime = time;
        lastE = e;
    }
    public void derivUpdate(double time, double d, double... x) {
        if (!Double.isNaN(d)) {
            this.d = d;
        }
        e = setPoint - x[0];
        if (!Double.isNaN(lastTime)) {
            i += (e + lastE) * (time - lastTime) / 2;
        }
        f = kf.applyAsDouble(x);
        lastTime = time;
        lastE = e;
    }
}