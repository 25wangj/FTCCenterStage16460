package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.AsymProfile;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
public class TurnTrajectory implements Trajectory {
    private double ti;
    private Vec pos;
    private MotionProfile profile;
    public TurnTrajectory(AsymConstraints constraints, Pose pos, double h) {
        this.pos = pos.vec();
        profile = new AsymProfile(constraints, ti, pos.h, 0, h, 0);
    }
    @Override
    public TrajectoryState state(double t) {
        return new TrajectoryState(new Pose(pos, profile.pos(t - ti)),
                new Pose(0, 0, profile.vel(t - ti)), new Vec(0, 0));
    }
    @Override
    public void setTi(double ti) {
        this.ti = ti;
    }
    @Override
    public double tf() {
        return profile.tf() + ti;
    }
    public double[] tfs() {
        return new double[] {0, profile.tf()};
    }
}
