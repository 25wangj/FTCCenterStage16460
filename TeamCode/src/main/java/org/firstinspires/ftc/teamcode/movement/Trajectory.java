package org.firstinspires.ftc.teamcode.movement;
public interface Trajectory {
    Pose pos(double t);
    Pose vel(double t);
    Vec accel(double t);
    void setTi(double ti);
    double tf();
}
