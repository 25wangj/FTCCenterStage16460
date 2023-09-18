package org.firstinspires.ftc.teamcode.sensors;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Encoder {
    public enum Direction {FORWARDS, REVERSE}
    private DcMotorEx motor;
    private ElapsedTime clock;
    private double start;
    private Direction dir;
    public void setDirection(Direction direction) {
        dir = direction;
    }
    public Direction getDirection() {
        return dir;
    }
    private int mult() {
        return ((dir == Direction.FORWARDS) == (motor.getDirection() == DcMotorSimple.Direction.FORWARD)) ? 1 : -1;
    }
    public void synch(double time) {
        start = clock.seconds() - time;
    }
}
