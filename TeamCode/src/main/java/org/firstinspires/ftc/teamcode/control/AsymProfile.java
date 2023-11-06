package org.firstinspires.ftc.teamcode.control;
import static java.lang.Math.*;
public class AsymProfile extends MotionProfile {
    private AsymConstraints c;
    private double t1;
    private double t2;
    private int sgn;
    public AsymProfile(AsymConstraints c, double ti, double xi, double vi, double xf, double vf) {
        this.ti = ti;
        this.xi = xi;
        this.vi = vi;
        this.xf = xf;
        this.vf = vf;
        this.c = c;
        sgn = (xf < xi) ? -1 : 1;
        double v2 = Math.sqrt((2 * c.ai * c.af * abs(xf - xi) + c.af * vi * vi + c.ai * vf * vf) / (c.ai + c.af));
        if (sgn == 1 && v2 < max(vi, vf) || sgn == -1 && -v2 > min(vi, vf)) {
            throw new IllegalArgumentException("Impossible profile");
        }
        if (v2 > c.vm) {
            tf = ti + (2 * abs(xf - xi) + (sgn * c.vm - vi) * (sgn * c.vm - vi) / c.ai +
                    (sgn * c.vm - vf) * (sgn * c.vm - vf) / c.af) / (2 * c.vm);
            t1 = ti + abs(sgn * c.vm - vi) / c.ai;
            t2 = tf - abs(sgn * c.vm - vf) / c.af;
        } else {
            tf = ti + abs(sgn * v2 - vi) / c.ai + abs(sgn * v2 - vf) / c.af;
            t1 = ti + abs(sgn * v2 - vi) / c.ai;
            t2 = t1;
        }
    }
    @Override
    public double getX(double t) {
        if (t < ti) {
            return xi + vi * (t - ti);
        } else if (t < t1) {
            return xi + vi * (t - ti) + sgn * c.ai * (t - ti) * (t - ti) / 2;
        } else if (t < t2) {
            return xi + vi * (t1 - ti) + sgn * c.ai * (t1 - ti) * (t1 - ti) / 2 + sgn * c.vm * (t - t1);
        } else if (t < tf) {
            return xf + vf * (t - tf) - sgn * c.af * (t - tf) * (t - tf) / 2;
        }
        return xf + vf * (t - tf);
    }
    @Override
    public double getV(double t) {
        if (t < ti) {
            return vi;
        } else if (t < t1) {
            return vi + sgn * c.ai * (t - t1);
        } else if (t < t2) {
            return sgn * c.vm;
        } else if (t < tf) {
            return vf + sgn * c.ai * (t - tf);
        }
        return vf;
    }
    @Override
    public double getA(double t) {
        if (t > ti && t <= t1) {
            return sgn * c.ai;
        } else if (t > t2 && t < tf) {
            return -sgn * c.af;
        }
        return 0;
    }
    public static AsymProfile extendAsym(MotionProfile p, AsymConstraints c, double ti, double xf, double vf) {
        return new AsymProfile(c, ti, p.getX(ti), p.getV(ti), xf, vf);
    }
    public static AsymProfile extendAsym(MotionProfile p, AsymConstraints c, double xf, double vf) {
        return extendAsym(p, c, p.tf, xf, vf);
    }
}
