package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
public class Trajectory {
    private Path path;
    private MotionProfile moveProfile;
    private MotionProfile turnProfile;
    private double len;
    private double hi;
    private double ti;
    public Trajectory(Path path, AsymConstraints moveConstraints, double vi, double vf, double hi) {
        this.path = path;
        len = path.length();
        moveProfile = new AsymProfile(moveConstraints, 0, 0, vi, len, vf);
        this.hi = hi - path.vel(0).angle();
    }
    public Trajectory(Path path, AsymConstraints moveConstraints, AsymConstraints turnConstraints, double vi, double vf, double hi, double hf) {
        this.path = path;
        moveProfile = new AsymProfile(moveConstraints.scale(1 / len), 0, 0, vi / len, 1, vf / len);
        turnProfile = new AsymProfile(turnConstraints, 0, hi, 0, hf, 0);
    }
    public void setTi(double ti) {
        this.ti = ti;
    }
    public Pose pos(double t) {
        double x = moveProfile.pos(t - ti) / path.length();
        double h;
        if (turnProfile == null) {
            h = hi + path.vel(x).angle();
        } else {
            h = turnProfile.pos(t - ti);
        }
        return new Pose(path.pos(x), h);
    }
    public Pose vel(double t) {
        double x = moveProfile.pos(t - ti);
        double v = moveProfile.vel(t - ti);
        double a;
        if (turnProfile == null) {
            a = path.angVel(x) * v;
        } else {
            a = turnProfile.vel(x);
        }
        return new Pose(path.vel(x).mult(v), a);
    }
    public Vec accel(double t) {
        double x = moveProfile.pos(t - ti);
        double v = moveProfile.vel(t - ti);
        return path.vel(x).combo(moveProfile.accel(t - ti), path.accel(x), v * v);
    }
    public double tf() {
        if (turnProfile == null) {
            return ti + moveProfile.tf();
        }
        return ti + max(moveProfile.tf(), turnProfile.tf());
    }
}