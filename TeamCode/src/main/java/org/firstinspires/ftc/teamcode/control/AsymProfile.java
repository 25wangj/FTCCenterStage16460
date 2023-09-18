package org.firstinspires.ftc.teamcode.control;

public class AsymProfile extends MotionProfile {
    double vm;
    double am1;
    double am2;
    boolean flat;
    int sgn;
    public AsymProfile(double vm, double am1, double am2, double ti, double xi, double vi, double xf, double vf) {

    }
    @Override
    public double getX(double t) {

    }
    @Override
    public double getV(double t) {

    }
    @Override
    public double getA(double t) {

    }
    public AsymProfile extendAsym(MotionProfile p, double vm, double am1, double am2, double ti, double xf, double vf) {
        return new AsymProfile(vm, am1, am2, ti, p.getX(ti), p.getV(ti), xf, vf);
    }
    public AsymProfile extendAsym(MotionProfile p, double vm, double am1, double am2, double xf, double vf) {
        return extendAsym(p, vm, am1, am2, p.getTf(), xf, vf);
    }
}
