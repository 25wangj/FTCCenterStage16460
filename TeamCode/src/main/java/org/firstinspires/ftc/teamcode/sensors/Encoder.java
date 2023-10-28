package org.firstinspires.ftc.teamcode.sensors;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class Encoder {
    public enum Direction {FORWARDS, REVERSE}
    private DcMotorEx motor;
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
}
