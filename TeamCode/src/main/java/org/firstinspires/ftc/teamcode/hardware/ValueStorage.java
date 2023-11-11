package org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.movement.Pose;
public class ValueStorage {
    public enum Side {
        RED, BLUE
    }
    public static Side lastSide = Side.RED;
    public static Pose lastPose = new Pose(0, 0, 0);
}
