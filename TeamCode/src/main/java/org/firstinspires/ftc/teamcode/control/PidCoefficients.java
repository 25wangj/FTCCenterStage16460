package org.firstinspires.ftc.teamcode.control;
public class PidCoefficients {
    public final double kp;
    public final double ki;
    public final double kd;
    public PidCoefficients(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
}