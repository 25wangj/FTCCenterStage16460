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
        this(v.getX(), v.getY(), h);
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getH() {
        return h;
    }
    public Vec vec() {
        return new Vec(x, y);
    }
}
