package org.firstinspires.ftc.teamcode.control;
public class SymProfile extends AsymProfile {
    public SymProfile(SymConstraints constraints, double ti, double xi, double vi, double xf, double vf) {
        super(constraints, ti, xi, vi, xf, vf);
    }
    public static SymProfile extendSym(MotionProfile p, SymConstraints constraints, double t, double xf, double vf) {
        return new SymProfile(constraints, t, p.pos(t), p.vel(t), xf, vf);
    }
    public static SymProfile extendSym(MotionProfile p, SymConstraints constraints, double xf, double vf) {
        return extendSym(p, constraints, p.tf, xf, vf);
    }
}