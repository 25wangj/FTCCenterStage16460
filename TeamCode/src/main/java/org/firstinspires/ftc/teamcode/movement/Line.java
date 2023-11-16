package org.firstinspires.ftc.teamcode.movement;
public class Line implements Path {
    private Vec start;
    private Vec dir;
    public Line(Vec start, Vec end) {
        this.start = start;
        dir = end.combo(1, start, -1);
    }
    @Override
    public PathState state(double t) {
        return new PathState(start.combo(1, dir, t), dir, 0);
    }
    @Override
    public double length() {
        return dir.norm();
    }
    public static Line extendX(Vec pos, Vec dir, double x) {
        if (dir.x == 0 || dir.x * (x - pos.x) < 0) {
            throw new IllegalArgumentException("Extension impossible");
        }
        return new Line(pos, new Vec(x, pos.y + dir.y / dir.x * (x - pos.x)));
    }
    public static Line extendY(Vec pos, Vec dir, double y) {
        if (dir.y == 0 || dir.y * (y - pos.y) < 0) {
            throw new IllegalArgumentException("Extension impossible");
        }
        return new Line(pos, new Vec(pos.x + dir.x / dir.y * (y - pos.y), y));
    }
}