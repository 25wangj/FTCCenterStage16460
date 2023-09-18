package org.firstinspires.ftc.teamcode.control;
public abstract class MotionProfile {
    protected double xi;
    protected double vi;
    protected double ti;
    protected double xf;
    protected double tf;
    protected double vf;
    public abstract double getX(double t);
    public abstract double getV(double t);
    public abstract double getA(double t);
    public double getTf() {
        return tf;
    }
    public double getTi() {
        return ti;
    }
}
