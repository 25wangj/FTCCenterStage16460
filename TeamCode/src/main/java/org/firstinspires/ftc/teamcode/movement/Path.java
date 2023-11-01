package org.firstinspires.ftc.teamcode.movement;
public interface Path {
    Vec pos(double t);
    Vec vel(double t);
    Vec curvature(double t);
    double length();
}
