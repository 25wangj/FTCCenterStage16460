package org.firstinspires.ftc.teamcode.movement;
public interface Path {
    Vec pos(double t);
    Vec dir(double t);
    double angVel(double t);
    Vec curvature(double t);
    double length();
}