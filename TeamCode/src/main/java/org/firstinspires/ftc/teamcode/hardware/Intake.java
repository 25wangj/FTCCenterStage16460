package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.command.Subsystem;
public class Intake implements Subsystem {
    public static final double rollerUp = 0;
    public static final double rollerDown = 0;
    public static final double gateClosed = 0.45;
    public static final double gateOpen = 0.73;
    public static final double gateUp = 0.87;
    private DcMotorEx intake;
    private Servo roller;
    private Servo gate;
    public Intake(LinearOpMode opMode) {
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        roller = opMode.hardwareMap.get(Servo.class, "roller");
        gate = opMode.hardwareMap.get(Servo.class, "gate");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void update(double time, boolean active) {}
}
