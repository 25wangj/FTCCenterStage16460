package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
public class Vec {
    private double x;
    private double y;
    public Vec(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public double x() {
        return x;
    }
    public double y() {
        return y;
    }
    public double norm() {
        return Math.sqrt(x * x + y * y);
    }
    public Vec normalize() {
        return mult(1 / norm());
    }
    public Vec mult(double a) {
        return new Vec(a * x, a * y);
    }
    public Vec combo(double a, Vec other, double b) {
        return new Vec(a * x + b * other.x, a * y + b * other.y);
    }
    public Vec rotate(double a) {
        return new Vec(x * cos(a) - y * sin(a), x * sin(a) + y * cos(a));
    }
    public double angle() {
        return atan2(y, x);
    }
}