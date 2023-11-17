package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
public class Intake implements Subsystem {
    public static final double rollerUp = 0.26;
    public static final double rollerDown = 0.37;
    public static final double gateClosed = 0.45;
    public static final double gateOpen = 0.72;
    public static final double gatePush = 0.64;
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
    public Command twiddle() {
        return new SeqCommand(new WaitCommand(t -> setGate(gatePush), 0.15, this),
                new WaitCommand(t -> setGate(0.8), 0.25, this),
                new WaitCommand(t -> setGate(gateOpen), 0.1, this));
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
