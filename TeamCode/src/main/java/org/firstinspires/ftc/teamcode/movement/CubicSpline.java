package org.firstinspires.ftc.teamcode.movement;
public class CubicSpline {
    public final double xi;
    public final double vi;
    public final double xf;
    public final double vf;
    public CubicSpline(double xi, double vi, double xf, double vf) {
        this.xi = xi;
        this.vi = vi;
        this.xf = xf;
        this.vf = vf;
    }
    public double pos(double t) {
        return xi * (2 * t * t * t - 3 * t * t + 1) + vi * (t * t * t - 2 * t * t + t) +
                xf * (-2 * t * t * t + 3 * t * t) + vf * (t * t * t - t * t);
    }
    public double vel(double t) {
        return xi * (6 * t * t - 6 * t) + vi * (3 * t * t - 4 * t + 1) + xf * (-6 * t * t + 6 * t) + vf * (3 * t * t - 2 * t);
    }
    public double accel(double t) {
        return xi * (12 * t - 6) + vi * (6 * t - 4) + xf * (-12 * t + 6) + vf * (6 * t - 2);
    }
}