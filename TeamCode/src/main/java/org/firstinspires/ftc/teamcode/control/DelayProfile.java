package org.firstinspires.ftc.teamcode.control;
public class DelayProfile extends MotionProfile {
    public DelayProfile(double ti, double xi, double vi, double dt) {
        this.ti = ti;
        this.xi = xi;
        this.vi = vi;
        this.tf = ti + dt;
        this.xf = xi + vi * dt;
        this.vf = vi;
    }
    @Override
    public double pos(double t) {
        return xi + vi * (t - ti);
    }
    @Override
    public double vel(double t) {
        return vi;
    }
    @Override
    public double accel(double t) {
        return 0;
    }
    public static DelayProfile extendDelay(MotionProfile p, double ti, double dt) {
        return new DelayProfile(ti, p.pos(ti), p.vel(ti), dt);
    }
    public static DelayProfile extendDelay(MotionProfile p, double dt) {
        return extendDelay(p, p.tf, dt);
    }
}
