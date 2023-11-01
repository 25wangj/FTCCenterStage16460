package org.firstinspires.ftc.teamcode.movement;
public class Line implements Path {
    private Vec start;
    private Vec dir;
    public Line(Vec start, Vec end) {
        this.start = start;
        dir = end.combo(1, start, -1);
    }
    @Override
    public Vec pos(double t) {
        return start.combo(1, dir, t);
    }
    @Override
    public Vec vel(double t) {
        return dir;
    }
    @Override
    public Vec curvature(double t) {
        return new Vec(0, 0);
    }
    @Override
    public double length() {
        return dir.norm();
    }
}
