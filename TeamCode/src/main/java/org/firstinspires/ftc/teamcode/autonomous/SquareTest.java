package org.firstinspires.ftc.teamcode.autonomous;
import static java.lang.Math.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RepeatCommand;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.TrajCommandBuilder;
import org.firstinspires.ftc.teamcode.movement.Vec;
@Config
@Autonomous(name = "SquareTest")
public class SquareTest extends CommandOpMode {
    public static double dist = 48;
    private MecanumDrive drive;
    @Override
    public void initOpMode() {
        drive = new MecanumDrive(this, true);
        scheduler.register(drive);
        Command traj = new TrajCommandBuilder(drive, new Pose(0, 0, 0))
                .lineTo(new Vec(dist, 0))
                .pause(0.25)
                .turn(PI / 2)
                .lineTo(new Pose(dist, dist, PI))
                .pause(0.25)
                .lineTo(new Vec(0, dist))
                .pause(0.25)
                .turn(3 * PI / 2)
                .lineTo(new Pose(0, 0, 2 * PI))
                .pause(0.25)
                .build(scheduler);
        scheduler.schedule(new RepeatCommand(traj));
        scheduler.schedule(FnCommand.repeat(t -> {
            Pose p = drive.pose();
            Pose ap = drive.getTrajectory().pos(t);
            telemetry.addData("X", p.x);
            telemetry.addData("Y", p.y);
            telemetry.addData("Heading", p.h % (2 * PI));
            telemetry.addData("Target X", ap.x);
            telemetry.addData("Target Y", ap.y);
            telemetry.addData("Target heading", ap.h % (2 * PI));
        }));
    }
}
