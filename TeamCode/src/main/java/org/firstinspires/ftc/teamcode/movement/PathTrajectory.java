package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.DelayProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
public class PathTrajectory implements Trajectory {
    private Path path;
    private MotionProfile moveProfile;
    private MotionProfile turnProfile;
    private double len;
    private double hi;
    private double ti;
    public PathTrajectory(Path path, AsymConstraints moveConstraints, double vi, double vf, double hi) {
        this.path = path;
        len = path.length();
        moveProfile = new AsymProfile(moveConstraints.scale(1 / len), 0, 0, vi / len, 1, vf / len);
        System.out.println("Constructed end time " + moveProfile.tf());
        this.hi = hi - path.vel(0).angle();
    }
    public PathTrajectory(Path path, AsymConstraints moveConstraints, AsymConstraints turnConstraints, double vi, double vf, double hi, double hf) {
        this.path = path;
        len = path.length();
        moveProfile = new AsymProfile(moveConstraints.scale(1 / len), 0, 0, vi / len, 1, vf / len);
        turnProfile = new AsymProfile(turnConstraints, 0, hi, 0, hf, 0);
    }
    @Override
    public Pose pos(double t) {
        double x = moveProfile.pos(t - ti);
        //System.out.println("Parameter " + x);
        double h;
        if (turnProfile == null) {
            h = hi + path.vel(x).angle();
        } else {
            h = turnProfile.pos(t - ti);
        }
        return new Pose(path.pos(x), h);
    }
    @Override
    public Pose vel(double t) {
        double x = moveProfile.pos(t - ti);
        double v = moveProfile.vel(t - ti);
        double av;
        if (turnProfile == null) {
            av = path.angVel(x) * v;
        } else {
            av = turnProfile.vel(x);
        }
        return new Pose(path.vel(x).mult(v), av);
    }
    @Override
    public Vec accel(double t) {
        double x = moveProfile.pos(t - ti);
        double v = moveProfile.vel(t - ti);
        return path.vel(x).combo(moveProfile.accel(t - ti), path.accel(x), v * v);
    }
    @Override
    public void setTi(double ti) {
        this.ti = ti;
    }
    @Override
    public double tf() {
        if (turnProfile == null) {
            return ti + moveProfile.tf();
        }
        return ti + max(moveProfile.tf(), turnProfile.tf());
    }
}