package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Double.isNaN;
import static java.lang.Math.*;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
public class PathTrajectory implements Trajectory {
    private Path[] paths;
    private AsymProfile moveProfile;
    private AsymProfile[] turnProfiles;
    private double[] lens;
    private double len;
    private double[] hs;
    private double[] tfs;
    private double ti;
    public PathTrajectory(Path[] paths, double[] hs, AsymConstraints moveConstraints,
                          AsymConstraints turnConstraints, double vi, double vf) {
        this.paths = paths;
        lens = new double[paths.length + 1];
        this.hs = hs;
        turnProfiles = new AsymProfile[paths.length + 1];
        for (int i = 0; i < paths.length; i++) {
            lens[i + 1] = lens[i] + paths[i].length();
            if (isNaN(hs[i + 1])) {
                hs[i + 1] = hs[i] + paths[i].state(1).dir.angle() - paths[i].state(0).dir.angle();
            } else {
                turnProfiles[i] = new AsymProfile(turnConstraints, 0, hs[i], 0, hs[i + 1], 0);
            }
        }
        len = lens[paths.length];
        moveProfile = new AsymProfile(moveConstraints, 0, 0, vi, len, vf);
        double[] fs = new double[paths.length];
        double max = 1;
        for (int i = 0; i < paths.length; i++) {
            if (turnProfiles[i] != null) {
                fs[i] = turnProfiles[i].tf() / (t(lens[i + 1]) - t(lens[i]));
                max = max(max, fs[i]);
            }
        }
        moveProfile = new AsymProfile(moveConstraints.scaleT(1 / max), 0, 0, vi, len, vf);
        tfs = new double[paths.length + 1];
        for (int i = 0; i < paths.length; i++) {
            tfs[i + 1] = t(lens[i + 1]);
            if (fs[i] != 0) {
                turnProfiles[i] = new AsymProfile(turnConstraints.scaleT(fs[i] / max), tfs[i], hs[i], 0, hs[i + 1], 0);
            }
        }
    }
    @Override
    public TrajectoryState state(double t) {
        double x = moveProfile.pos(t - ti);
        double v = moveProfile.vel(t - ti);
        int ind = 0;
        for (int i = 0; i < paths.length; i++) {
            if (t - ti < tfs[i + 1]) {
                ind = i;
                break;
            }
        }
        PathState state = paths[ind].state((x - lens[ind]) / (lens[ind + 1] - lens[ind]));
        double h;
        double av;
        if (turnProfiles[ind] == null) {
            h = hs[ind] + state.dir.angle() - paths[ind].state(0).dir.angle();
            av = state.curv * v;
        } else {
            h = turnProfiles[ind].pos(t - ti);
            av = turnProfiles[ind].vel(t - ti);
        }
        return new TrajectoryState(new Pose(state.pos, h), new Pose(state.dir.mult(v), av),
                state.dir.combo(moveProfile.accel(t - ti), new Vec(state.dir.y, -state.dir.x), state.curv * v * v));
    }
    private double t(double x) {
        if (x < moveProfile.pos(moveProfile.t1())) {
            return sqrt(2 * x / moveProfile.c().ai);
        } else if (x < moveProfile.pos(moveProfile.t2())) {
            return moveProfile.t1() + (x - moveProfile.pos(moveProfile.t1())) / moveProfile.c().vm;
        } else {
            return moveProfile.tf() - sqrt(2 * (moveProfile.pos(moveProfile.tf()) - x) / moveProfile.c().af);
        }
    }
    @Override
    public void setTi(double ti) {
        this.ti = ti;
    }
    @Override
    public double tf() {
        return ti + tfs[paths.length];
    }
    public double[] tfs() {
        return tfs;
    }
}