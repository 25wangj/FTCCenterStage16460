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
    public Pose pos(double t) {
        return new Pose(pos, profile.pos(t - ti));
    }
    @Override
    public Pose vel(double t) {
        return new Pose(0, 0, profile.vel(t - ti));
    }
    @Override
    public Vec accel(double t) {
        return new Vec(0, 0);
    }
    @Override
    public void setTi(double ti) {
        this.ti = ti;
    }
    @Override
    public double tf() {
        return profile.tf() + ti;
    }
}
