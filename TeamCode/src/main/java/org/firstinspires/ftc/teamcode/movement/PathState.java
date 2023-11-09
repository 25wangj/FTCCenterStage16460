package org.firstinspires.ftc.teamcode.movement;
public class PathState {
    public final Vec pos;
    public final Vec vel;
    public final double curv;
    public PathState(Vec pos, Vec vel, double curv) {
        this.pos = pos;
        this.vel = vel;
        this.curv = curv;
    }
}
