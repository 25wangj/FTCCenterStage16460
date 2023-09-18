package org.firstinspires.ftc.teamcode.control;
public class SymProfile extends AsymProfile {
    public SymProfile(double vm, double am, double ti, double xi, double vi, double xf, double vf) {
        super(vm, am, am, ti, xi, vi, xf, vf);
    }
    public static SymProfile extendSym(MotionProfile p, double vm, double am, double t, double xf, double vf) {
        return new SymProfile(vm, am, t, p.getX(t), p.getV(t), xf, vf);
    }
    public static SymProfile extendSym(MotionProfile p, double vm, double am, double xf, double vf) {
        return extendSym(p, vm, am, p.getTf(), xf, vf);
    }
}