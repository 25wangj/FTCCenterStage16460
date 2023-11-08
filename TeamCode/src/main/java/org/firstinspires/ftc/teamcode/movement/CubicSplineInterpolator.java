package org.firstinspires.ftc.teamcode.movement;
public class CubicSplineInterpolator {
    private double[] t;
    private double xi;
    private double vi;
    private double xf;
    private double vf;
    private CubicSpline[] splines;
    public CubicSplineInterpolator(double[] t, double[] x, double[] v) {
        this.t = x;
        vi = v[0];
        vf = v[x.length - 1];
        splines = new CubicSpline[x.length - 1];
        for (int i = 0; i < splines.length; i++) {
            splines[i] = new CubicSpline(x[i], v[i], x[i + 1], v[i + 1]);
        }
    }
    public double get(double time) {
        if (time < t[0]) {
            return xi + vi * (time - t[0]);
        }
        for (int i = 1; i < t.length; i++) {
            if (time < t[i]) {
                return splines[i - 1].pos(time);
            }
        }
        return xf + vf * (time - t[t.length - 1]);
    }
}