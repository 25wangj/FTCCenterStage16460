package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.hardware.MecanumDrive.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.SymConstraints;
import org.firstinspires.ftc.teamcode.control.SymProfile;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.movement.Pose;
@Config
@Autonomous(name = "TurnFeedforwardTest")
public class TurnFeedforwardTest extends CommandOpMode {
    public static double width = 12.5;
    public static double angle = PI;
    public static double vm = 6;
    public static double am = 6;
    private MotionProfile profile;
    private MecanumDrive drive;
    @Override
    public void initOpMode() {
        drive = new MecanumDrive(this, true);
        scheduler.register(drive);
        scheduler.schedule(FnCommand.repeat(t -> {
            if (profile == null || t > profile.tf() + 1) {
                profile = new SymProfile(new SymConstraints(vm, am), t, 0, 0, angle, 0);
            }
            drive.setPowers(0, 0, driveKv * profile.vel(t) * width);
            Pose p = drive.pose();
            Pose v = drive.vel();
            telemetry.addData("X", p.x);
            telemetry.addData("Y", p.y);
            telemetry.addData("Heading", p.h % (2 * PI));
            telemetry.addData("Actual angular velocity", v.h);
            telemetry.addData("Target angular velocity", profile.vel(t));
        }));
    }
}