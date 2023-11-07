package org.firstinspires.ftc.teamcode.movement;
public class Pose {
    public final double x;
    public final double y;
    public final double h;
    public Pose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Pose(Vec v, double h) {
        this(v.x, v.y, h);
    }
    public Vec vec() {
        return new Vec(x, y);
    }
}
