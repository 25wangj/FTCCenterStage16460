package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
public class Trajectory {
    private Path path;
    private MotionProfile moveProfile;
    private MotionProfile turnProfile;
    private double len;
    private double hi;
    public Trajectory(Path path, AsymConstraints moveConstraints, double t, double vi, double vf, double hi) {
        this.path = path;
        len = path.length();
        moveProfile = new AsymProfile(moveConstraints, t, 0, vi, len, vf);
        this.hi = hi - path.dir(0).angle();
    }
    public Trajectory(Path path, AsymConstraints moveConstraints, AsymConstraints turnConstraints, double t, double vi, double vf, double hi, double hf) {
        this.path = path;
        moveProfile = new AsymProfile(moveConstraints, 0, 0, vi, len, vf);
        turnProfile = new AsymProfile(turnConstraints, t, hi, 0, hf, 0);
    }
    public Pose pos(double t) {
        double x = moveProfile.getX(t) / path.length();
        double h;
        if (turnProfile == null) {
            h = hi + path.dir(x).angle();
        } else {
            h = turnProfile.getX(t);
        }
        return new Pose(path.pos(x), h);
    }
    public Pose vel(double t) {
        double x = moveProfile.getX(t) / len;
        double a;
        if (turnProfile == null) {
            a = path.angVel(x);
        } else {
            a = turnProfile.getV(x);
        }
        return new Pose(path.dir(x).mult(moveProfile.getV(x)), a);
    }
    public Vec accel(double t) {
        return path.dir(moveProfile.getX(t) / len).mult(moveProfile.getA(t));
    }
}