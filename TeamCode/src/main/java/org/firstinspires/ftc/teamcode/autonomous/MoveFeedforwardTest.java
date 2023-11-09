package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.control.MotionProfile;
import org.firstinspires.ftc.teamcode.control.SymConstraints;
import org.firstinspires.ftc.teamcode.control.SymProfile;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Config
@Autonomous(name = "MoveFeedforwardTest")
public class MoveFeedforwardTest extends CommandOpMode {
    public static double dist = 96;
    public static double kv = 0;
    public static double ka = 0;
    public static double vm = 60;
    public static double am = 60;
    private boolean forwards = true;
    private MotionProfile profile;
    private MecanumDrive drive;
    @Override
    public void initOpMode() {
        drive = new MecanumDrive(this, true);
        scheduler.register(drive);
        scheduler.schedule(FnCommand.repeat(t -> {
            if (profile == null || t > profile.tf() + 1) {
                if (forwards) {
                    profile = new SymProfile(new SymConstraints(vm, am), t, 0, 0, dist, 0);
                } else {
                    profile = new SymProfile(new SymConstraints(vm, am), t, dist, 0, 0, 0);
                }
                forwards = !forwards;
            }
            drive.setPowers(new Vec(kv * profile.vel(t) + ka * profile.accel(t), 0), 0);
            Pose p = drive.pose();
            Pose v = drive.vel();
            telemetry.addData("X", p.x);
            telemetry.addData("Y", p.y);
            telemetry.addData("Heading", p.h % (2 * PI));
            telemetry.addData("Actual velocity", v.vec().rotate(-p.h).x);
            telemetry.addData("Target velocity", profile.vel(t));
        }));
    }
}
