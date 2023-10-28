package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Listener;
import org.firstinspires.ftc.teamcode.hardware.Robot;
@TeleOp(name = "OneDriver")
public class TeleopOneDriver extends CommandOpMode {
    Robot robot;
    @Override
    public void initOpMode() {
        robot = new Robot(this, false);
        scheduler.addListener(Listener.risingEdge(() -> gamepad1.ps, FnCommand.once(() -> robot.drive.setHeading(0))));
        scheduler.schedule(FnCommand.repeat(() -> {
            double h = robot.drive.getHeading();
            double x = gamepad1.left_stick_x * cos(h) - gamepad1.left_stick_y * sin(h);
            double y = gamepad1.left_stick_x * sin(h) + gamepad1.left_stick_y * cos(h);
            double t = gamepad1.right_stick_x;
            if (abs(x) + abs(y) + abs(t) < 0.05) {
                robot.drive.setPowers(0, 0, 0);
            } else {
                robot.drive.setPowers(x, y, t);
            }
        }, robot.drive));
    }
}
