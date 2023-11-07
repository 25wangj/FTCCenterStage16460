package org.firstinspires.ftc.teamcode.control;
public abstract class MotionProfile {
    protected double xi;
    protected double vi;
    protected double ti;
    protected double xf;
    protected double tf;
    protected double vf;
    public abstract double pos(double t);
    public abstract double vel(double t);
    public abstract double accel(double t);
    public double tf() {
        return tf;
    }
    public double ti() {
        return ti;
    }
}
