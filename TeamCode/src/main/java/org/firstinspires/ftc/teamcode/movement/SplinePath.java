package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
public class SplinePath implements Path {
    public static final int APPROX_PTS = 5;
    public static final double EPS = 1e-6;
    private double len;
    private CubicSpline x;
    private CubicSpline y;
    private CubicSplineInterpolator arcLen;
    public SplinePath(Vec xi, Vec vi, Vec xf, Vec vf) {
        x = new CubicSpline(xi.x, vi.x, xf.x, vf.x);
        y = new CubicSpline(xi.y, vi.y, xf.y, vf.y);
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
    @Override public PathState state(double t) {
        double ta = arcLen.get(t * len);
        double xV = x.vel(ta);
        double yV = y.vel(ta);
        return new PathState(new Vec(x.pos(ta), y.pos(ta)), new Vec(xV, yV).normalize(),
                (xV * y.accel(ta) - x.accel(ta) * yV) / pow(xV * xV + yV * yV, 1.5));
    }
    @Override
    public double length() {
        return len;
    }
}