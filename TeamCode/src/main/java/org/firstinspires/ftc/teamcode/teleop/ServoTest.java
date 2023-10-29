package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Listener;
import org.firstinspires.ftc.teamcode.command.Subsystem;
@TeleOp(name = "ServoTest")
public class ServoTest extends CommandOpMode {
    private Servo servo;
    private double pos;
    @Override
    public void initOpMode() {
        servo = hardwareMap.get(Servo.class, "roller");
        pos = 0.5;
        Subsystem servoSubsystem = (t, b) -> {
            servo.setPosition(pos);
            telemetry.addData("Position", pos);
        };
        scheduler.register(servoSubsystem);
        scheduler.addListener(
            Listener.risingEdge(() -> gamepad1.a, FnCommand.once(t -> pos = min(pos + 0.1, 1))),
            Listener.risingEdge(() -> gamepad1.b, FnCommand.once(t -> pos = max(pos - 0.1, 0))),
            Listener.risingEdge(() -> gamepad1.x, FnCommand.once(t -> pos = min(pos + 0.01, 1))),
            Listener.risingEdge(() -> gamepad1.y, FnCommand.once(t -> pos = max(pos - 0.01, 0))));
    }
}