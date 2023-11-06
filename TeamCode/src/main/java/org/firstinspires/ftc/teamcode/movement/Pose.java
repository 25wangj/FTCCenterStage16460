package org.firstinspires.ftc.teamcode.movement;
public class Pose {
    private double x;
    private double y;
    private double h;
    public Pose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Pose(Vec v, double h) {
        this(v.x(), v.y(), h);
    }
    public double x() {
        return x;
    }
    public double y() {
        return y;
    }
    public double h() {
        return h;
    }
    public Vec vec() {
        return new Vec(x, y);
    }
}
