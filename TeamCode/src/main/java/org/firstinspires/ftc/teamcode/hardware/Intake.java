package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.command.Subsystem;
public class Intake implements Subsystem {
    public static final double rollerUp = 0;
    public static final double rollerDown = 0.05;
    public static final double gateClosed = 0.45;
    public static final double gateOpen = 0.73;
    public static final double gatePush = 0.68;
    public static final double gateUp = 0.87;
    public static final double intakeEject = -1;
    public static final double intakeClosed = 0.7;
    public static final double intakeOpen = 1;
    private DcMotorEx intake;
    private Servo roller;
    private Servo gate;
    public Intake(LinearOpMode opMode) {
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        roller = opMode.hardwareMap.get(Servo.class, "roller");
        gate = opMode.hardwareMap.get(Servo.class, "gate");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setRoller(double pos) {
        roller.setPosition(pos);
    }
    public void setGate(double pos) {
        gate.setPosition(pos);
    }
    public void setPower(double power) {
        intake.setPower(power);
    }
    @Override
    public void update(double time, boolean active) {}
}
