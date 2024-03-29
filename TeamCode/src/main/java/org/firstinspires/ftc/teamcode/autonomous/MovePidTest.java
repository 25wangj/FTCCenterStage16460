package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Config
@Disabled
@Photon
@Autonomous(name = "MovePidTest")
public class MovePidTest extends CommandOpMode {
    public static double dist = 96;
    public static double vm = 60;
    public static double ai = 80;
    public static double af = 80;
    private MecanumDrive drive;
    @Override
    public void initOpMode() {
        drive = new MecanumDrive(this, true);
        scheduler.register(drive);
        Command traj = new TrajCommandBuilder(drive, new Pose(0, 0, 0))
                .setMoveConstraints(new AsymConstraints(vm, ai, af))
                .lineTo(new Vec(dist, 0))
                .pause(1)
                .lineTo(new Vec(0, 0))
                .pause(1)
                .build(scheduler);
        scheduler.schedule(new RepeatCommand(traj));
        scheduler.schedule(FnCommand.repeat(t -> {
            Pose p = drive.pose();
            Pose ap = drive.getTrajectory().state(t).pos;
            telemetry.addData("X", p.x);
            telemetry.addData("Y", p.y);
            telemetry.addData("Heading", p.h % (2 * PI));
            telemetry.addData("Target X", ap.x);
            telemetry.addData("Target Y", ap.y);
            telemetry.addData("Target heading", ap.h % (2 * PI));
        }));
    }
}
