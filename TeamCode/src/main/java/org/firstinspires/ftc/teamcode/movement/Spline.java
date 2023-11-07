package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
public class Spline implements Path {
    public static final int APPROX_PTS = 5;
    public static final double EPS = 1e-6;
    private double len;
    private CubicSpline x;
    private CubicSpline y;
    private CubicSplineInterpolator arcLen;
    private Spline(Vec xi, Vec vi, Vec xf, Vec vf) {
        x = new CubicSpline(xi.x, vi.x, xf.x, vf.x);
        y = new CubicSpline(xi.y, xi.y, xf.y, xf.y);
        double[] tArr = new double[APPROX_PTS + 1];
        double[] xArr = new double[APPROX_PTS + 1];
        double[] vArr = new double[APPROX_PTS + 1];
        AdaptiveQuadrature integrator = new AdaptiveQuadrature(t -> sqrt(x.vel(t) * x.vel(t) + y.vel(t) * y.vel(t)));
        for (int i = 0; i <= APPROX_PTS; i++) {
            double t = (double)i / APPROX_PTS;
            tArr[i] = integrator.integrate(0, t, EPS);
            xArr[i] = t;
            vArr[i] = 1 / sqrt(x.vel(t) * x.vel(t) + y.vel(t) * y.vel(t));
        }
        len = tArr[APPROX_PTS];
        arcLen = new CubicSplineInterpolator(tArr, xArr, vArr);
    }
    @Override
    public Vec pos(double t) {
        double ta = arcLen.get(t);
        return new Vec(x.pos(ta), y.pos(ta));
    }
    @Override
    public Vec vel(double t) {
        double ta = arcLen.get(t);
        return new Vec(x.vel(ta), y.vel(ta)).normalize().mult(len);
    }
    @Override
    public Vec accel(double t) {
        double ta = arcLen.get(t);
        double xV = x.vel(ta);
        double yV = y.vel(ta);
        double xA = x.accel(ta);
        double yA = y.accel(ta);
        double sqV = xV * xV + yV * yV;
        return new Vec(xA, yA).combo(1 / sqV, new Vec(xV, yV), -(xV * xA + yV * yA) / (sqV * sqV));
    }
    @Override
    public double angVel(double t) {
        double ta = arcLen.get(t);
        double xV = x.vel(ta);
        double yV = y.vel(ta);
        double xA = x.accel(ta);
        double yA = y.accel(ta);
        return len * (xA * yV - xV * yA) / pow(xV * xV + yV * yV, 1.5);
    }
    @Override
    public double length() {
        return len;
    }
}