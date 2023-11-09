package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Vec;

@Config
@TeleOp(name = "LateralDistTest")
public class LateralDistTest extends CommandOpMode {
    public static int turns = 10;
    private MecanumDrive drive;
    double hFinal = Double.NaN;
    @Override
    public void initOpMode() {
        drive = new MecanumDrive(this, true);
        scheduler.register(drive);
        scheduler.schedule(FnCommand.repeat(t -> {
            Vec v = new Vec(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            double turn = -gamepad1.right_stick_x;
            if (v.norm() + abs(turn) < 0.05) {
                drive.setPowers(new Vec(0, 0), 0);
            } else {
                drive.setPowers(v, turn);
            }
            Pose p = drive.pose();
            if (gamepad1.a && Double.isNaN(hFinal)) {
                hFinal = p.h;
            }
            if (Double.isNaN(hFinal)) {
                telemetry.addLine("Turn " + turns + "times and press A to finish");
                telemetry.addData("X", p.x);
                telemetry.addData("Y", p.y);
                telemetry.addData("Heading", p.h % (2 * PI));
            } else {
                telemetry.addData("Raw heading", hFinal);
                telemetry.addData("Predicted trackwidth", 2 * turns * PI * MecanumDrive.parDist / hFinal);
            }
        }));
    }
}