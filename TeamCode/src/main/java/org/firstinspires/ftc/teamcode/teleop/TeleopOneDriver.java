package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.ValueStorage.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.sensors.RisingEdgeDetector;
@TeleOp(name = "OneDriver")
public class TeleopOneDriver extends CommandOpMode {
    Robot robot;
    @Override
    public void initOpMode() {
        robot = new Robot(this, false);
        robot.drive.setHeading(lastPose.getH() + lastSide * PI / 2);
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.ps, FnCommand.once(t -> robot.drive.setHeading(0))));
        scheduler.schedule(FnCommand.repeat(t -> {
            double heading = robot.drive.getHeading();
            double x = gamepad1.left_stick_x * cos(heading) - gamepad1.left_stick_y * sin(heading);
            double y = gamepad1.left_stick_x * sin(heading) + gamepad1.left_stick_y * cos(heading);
            double turn = gamepad1.right_stick_x;
            if (abs(x) + abs(y) + abs(turn) < 0.05) {
                robot.drive.setPowers(0, 0, 0);
            } else {
                robot.drive.setPowers(x, y, turn);
            }
        }, robot.drive));
    }
}
