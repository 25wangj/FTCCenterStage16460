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
    public Vec dir(double t) {
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
    public static Line extendX(Vec pos, Vec dir, double x) {
        if (dir.getX() == 0 || dir.getX() * (pos.getX() - x) < 0) {
            throw new IllegalArgumentException("Extension impossible");
        }
        return new Line(pos, new Vec(x, pos.getY() + dir.getY() / dir.getX() * (pos.getX() - x)));
    }
    public static Line extendY(Vec pos, Vec dir, double y) {
        if (dir.getY() == 0 || dir.getY() * (pos.getY() - y) < 0) {
            throw new IllegalArgumentException("Extension impossible");
        }
        return new Line(pos, new Vec(pos.getX() + dir.getX() / dir.getY() * (pos.getY() - y), y));
    }
}
