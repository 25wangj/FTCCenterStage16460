package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Listener;
import org.firstinspires.ftc.teamcode.command.Subsystem;
public class ServoTest extends CommandOpMode {
    private Servo servo;
    private double pos;
    @Override
    public void initOpMode() {
        servo = hardwareMap.get(Servo.class, "test");
        pos = 0.5;
        Subsystem servoSubsystem = (t, b) -> {
            servo.setPosition(pos);
            telemetry.addData("Position", pos);
        };
        scheduler.register(servoSubsystem);
        scheduler.addListener(
            Listener.risingEdge(() -> gamepad1.a, FnCommand.once(() -> pos = min(pos + 0.1, 1))),
            Listener.risingEdge(() -> gamepad1.b, FnCommand.once(() -> pos = max(pos - 0.1, 0))),
            Listener.risingEdge(() -> gamepad1.x, FnCommand.once(() -> pos = min(pos + 0.01, 1))),
            Listener.risingEdge(() -> gamepad1.y, FnCommand.once(() -> pos = max(pos - 0.01, 0))));
    }
}