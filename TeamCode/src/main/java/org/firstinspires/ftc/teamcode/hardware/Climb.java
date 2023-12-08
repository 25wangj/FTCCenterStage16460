package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
public class Climb implements Subsystem {
    public static final double latchClosed = 0.17;
    public static final double latchHalf = 0.25;
    public static final double latchOpen = 0.33;
    public static final double planeHold = 0.35;
    public static final double planeRelease = 0.45;
    public static final int climbOut = -2500;
    public static final int climbUp = -1500;
    private DcMotorEx climb;
    private Servo latch;
    private Servo plane;
    public Climb(LinearOpMode opMode, boolean auto) {
        climb = opMode.hardwareMap.get(DcMotorEx.class, "climb");
        latch = opMode.hardwareMap.get(Servo.class, "latch");
        plane = opMode.hardwareMap.get(Servo.class, "plane");
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (auto) {
            climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        climb.setTargetPosition(0);
        climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        latch.setPosition(latchClosed);
        plane.setPosition(planeHold);
    }
    public Command release() {
        return new SeqCommand(
                new WaitCommand(t -> {
                    latch.setPosition(latchHalf);
                    climb.setPower(1);
                    climb.setTargetPosition(climbOut);}, 1),
                new WaitCommand(t -> latch.setPosition(latchOpen), 0.5));
    }
    public void climb() {
        climb.setTargetPosition(climbUp);
    }
    public void setLatch(double pos) {
        latch.setPosition(pos);
    }
    public void setPlane(double pos) {
        plane.setPosition(pos);
    }
    public int get() {
        return climb.getCurrentPosition();
    }
    @Override
    public void update(double time, boolean active) {
    }
}
