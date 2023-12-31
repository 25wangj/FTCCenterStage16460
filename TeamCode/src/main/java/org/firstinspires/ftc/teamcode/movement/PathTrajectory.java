package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
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
        moveProfile = new AsymProfile(moveConstraints.scaleX(1 / len), 0, 0, vi / len, 1, vf / len);
        this.hi = hi - path.state(0).vel.angle();
    }
    public PathTrajectory(Path path, AsymConstraints moveConstraints, AsymConstraints turnConstraints, double vi, double vf, double hi, double hf) {
        this.path = path;
        len = path.length();
        moveProfile = new AsymProfile(moveConstraints.scaleX(1 / len), 0, 0, vi / len, 1, vf / len);
        turnProfile = new AsymProfile(turnConstraints, 0, hi, 0, hf, 0);
        if (turnProfile.tf() != 0) {
            if (moveProfile.tf() > turnProfile.tf()) {
                AsymConstraints turnConstraints2 = turnConstraints.scaleT(turnProfile.tf() / moveProfile.tf());
                turnProfile = new AsymProfile(turnConstraints2, 0, hi, 0, hf, 0);
            } else {
                AsymConstraints moveConstraints2 = moveConstraints.scaleX(1 / len).scaleT(moveProfile.tf() / turnProfile.tf());
                moveProfile = new AsymProfile(moveConstraints2, 0, 0, vi / len, 1, vf / len);
            }
        }
    }
    @Override
    public TrajectoryState state(double t) {
        double x = moveProfile.pos(t - ti);
        double v = moveProfile.vel(t - ti);
        PathState state = path.state(x);
        double h;
        double av;
        if (turnProfile == null) {
            h = hi + state.vel.angle();
            av = state.curv * len * v;
        } else {
            h = turnProfile.pos(t - ti);
            av = turnProfile.vel(t - ti);
        }
        return new TrajectoryState(new Pose(state.pos, h), new Pose(state.vel.mult(v), av),
                state.vel.combo(moveProfile.accel(t - ti), new Vec(state.vel.y, -state.vel.x), state.curv * len * v * v));
    }
    @Override
    public void setTi(double ti) {
        this.ti = ti;
    }
    @Override
    public double tf() {
        return ti + moveProfile.tf();
    }
}