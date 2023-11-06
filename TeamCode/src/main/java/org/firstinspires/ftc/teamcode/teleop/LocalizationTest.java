package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.movement.Pose;
@TeleOp(name = "LocalizationTest")
public class LocalizationTest extends CommandOpMode {
    private MecanumDrive drive;
    @Override
    public void initOpMode() {
        drive = new MecanumDrive(this, true);
        scheduler.register(drive);
        scheduler.schedule(FnCommand.repeat(t -> {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            if (abs(x) + abs(y) + abs(turn) < 0.05) {
                drive.setPowers(0, 0, 0);
            } else {
                drive.setPowers(x, y, turn);
            }
            Pose p = drive.getPose();
            telemetry.addData("X", p.x());
            telemetry.addData("Y", p.y());
            telemetry.addData("Heading", p.h());
        }));
    }
}
