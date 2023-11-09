package org.firstinspires.ftc.teamcode.movement;
public class WaitTrajectory implements Trajectory {
    private double ti;
    private double seconds;
    private Pose pos;
    private Vec vel;
    public WaitTrajectory(Pose pos, Vec vel, double seconds) {
        this.seconds = seconds;
        this.pos = pos;
        this.vel = vel;
    }
    @Override
    public Pose pos(double t) {
        return new Pose(pos.vec().combo(1, vel, t - ti), pos.h);
    }
    @Override
    public Pose vel(double t) {
        return new Pose(vel, 0);
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
        return ti + seconds;
    }
}
