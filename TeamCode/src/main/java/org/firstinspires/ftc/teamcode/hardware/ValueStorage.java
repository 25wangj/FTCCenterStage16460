package org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.movement.Pose;
public class ValueStorage {
    public static class Side {
        public static final int RED = 1;
        public static final int BLUE = -1;
    }
    public static int lastSide = Side.RED;
    public static Pose lastPose = new Pose(0, 0, 0);
}
