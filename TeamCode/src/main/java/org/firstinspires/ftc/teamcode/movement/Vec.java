package org.firstinspires.ftc.teamcode.movement;
public class Vec {
    private double x;
    private double y;
    public Vec(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double norm() {
        return Math.sqrt(x * x + y * y);
    }
    public Vec mult(double a) {
        return new Vec(a * x, a * y);
    }
    public Vec combo(double a, Vec other, double b) {
        return new Vec(a * x + b * other.x, a * y + b * other.y);
    }
}