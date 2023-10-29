package org.firstinspires.ftc.teamcode.control;
import static java.lang.Math.*;
public class AsymProfile extends MotionProfile {
    protected double vm;
    protected double ai;
    protected double af;
    private double t1;
    private double t2;
    private int sgn;
    public AsymProfile(double vm, double ai, double af, double ti, double xi, double vi, double xf, double vf) {
        this.ti = ti;
        this.xi = xi;
        this.vi = vi;
        this.xf = xf;
        this.vf = vf;
        this.vm = vm;
        this.ai = ai;
        this.af = af;
        sgn = (xf < xi) ? -1 : 1;
        double v2 = Math.sqrt((2 * ai * af * abs(xf - xi) + af * vi * vi + ai * vf * vf) / (ai + af));
        if (sgn == 1 && v2 < max(vi, vf) || sgn == -1 && -v2 > min(vi, vf)) {
            throw new IllegalArgumentException("Impossible profile");
        }
        if (v2 > vm) {
            tf = ti + (2 * abs(xf - xi) + (sgn * vm - vi) * (sgn * vm - vi) / ai + (sgn * vm - vf) * (sgn * vm - vf) / af) / (2 * vm);
            t1 = ti + abs(sgn * vm - vi) / ai;
            t2 = tf - abs(sgn * vm - vf) / af;
        } else {
            tf = ti + abs(sgn * v2 - vi) / ai + abs(sgn * v2 - vf) / af;
            t1 = ti + abs(sgn * v2 - vi) / ai;
            t2 = t1;
        }
    }
    @Override
    public double getX(double t) {
        if (t < ti) {
            return xi + vi * (t - ti);
        } else if (t < t1) {
            return xi + vi * (t - ti) + sgn * ai * (t - ti) * (t - ti) / 2;
        } else if (t < t2) {
            return xi + vi * (t1 - ti) + sgn * ai * (t1 - ti) * (t1 - ti) / 2 + sgn * vm * (t - t1);
        } else if (t < tf) {
            return xf + vf * (t - tf) - sgn * af * (t - tf) * (t - tf) / 2;
        }
        return xf + vf * (t - tf);
    }
    @Override
    public double getV(double t) {
        if (t < ti) {
            return vi;
        } else if (t < t1) {
            return vi + sgn * ai * (t - t1);
        } else if (t < t2) {
            return sgn * vm;
        } else if (t < tf) {
            return vf + sgn * ai * (t - tf);
        }
        return vf;
    }
    @Override
    public double getA(double t) {
        if (t > ti && t <= t1) {
            return sgn * ai;
        } else if (t > t2 && t < tf) {
            return -sgn * af;
        }
        return 0;
    }
    public static AsymProfile extendAsym(MotionProfile p, double vm, double ai, double af, double ti, double xf, double vf) {
        return new AsymProfile(vm, ai, af, ti, p.getX(ti), p.getV(ti), xf, vf);
    }
    public static AsymProfile extendAsym(MotionProfile p, double vm, double ai, double af, double xf, double vf) {
        return extendAsym(p, vm, ai, af, p.tf, xf, vf);
    }
}
