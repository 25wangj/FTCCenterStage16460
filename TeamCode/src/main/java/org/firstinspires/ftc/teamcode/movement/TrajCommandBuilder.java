package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.SeqCommand;
import org.firstinspires.ftc.teamcode.command.WaitCommand;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import java.util.ArrayList;
public class TrajCommandBuilder {
    private Drivetrain drive;
    private Pose pos;
    private Vec tangent;
    private double vi;
    private double vf;
    private double tf;
    private AsymConstraints moveConstraints;
    private AsymConstraints turnConstraints;
    private ArrayList<Command> trajCommands;
    public TrajCommandBuilder(Drivetrain drive, Pose pos) {
        this.drive = drive;
        this.pos = pos;
        this.tangent = Vec.dir(pos.h);
        this.moveConstraints = drive.moveConstraints;
        this.turnConstraints = drive.turnConstraints;
        trajCommands = new ArrayList<>();
    }
    public void setTangent(double h) {
        tangent = Vec.dir(h);
    }
    public void setVf(double vf) {
        this.vf = vf;
    }
    public void setMoveConstraints(AsymConstraints moveConstraints) {
        this.moveConstraints = moveConstraints;
    }
    public void setTurnConstraints(AsymConstraints turnConstraints) {
        this.turnConstraints = turnConstraints;
    }
    public void resetConstraints() {
        moveConstraints = drive.moveConstraints;
        turnConstraints = drive.turnConstraints;
    }
    public void wait(double t) {
        pos = new Pose(pos.vec().combo(1, tangent, vi * t), pos.h);
        trajCommands.add(new WaitCommand(t));
    }
    public void turn(double h) {
        lineTo(new Pose(pos.vec(), h));
    }
    public void lineTo(Pose end) {
        trajCommands.add(trajCommand(new Line(pos.vec(), end.vec()), end.h));
    }
    public void lineTo(Vec end) {
        trajCommands.add(trajCommand(new Line(pos.vec(), end)));
    }
    public void lineToX(double x, double h) {
        trajCommands.add(trajCommand(Line.extendX(pos.vec(), tangent, x), h));
    }
    public void lineToX(double x) {
        trajCommands.add(trajCommand(Line.extendX(pos.vec(), tangent, x)));
    }
    public void lineToY(double y, double h) {
        trajCommands.add(trajCommand(Line.extendY(pos.vec(), tangent, y), h));
    }
    public void lineToY(double y) {
        trajCommands.add(trajCommand(Line.extendY(pos.vec(), tangent, y)));
    }
    public void splineTo(Pose end, double t, double v) {
        trajCommands.add(trajCommand(new Spline(pos.vec(), tangent.mult(v), end.vec(), Vec.dir(end.h).mult(v)), end.h));
    }
    public void splineTo(Vec end, double t, double v) {
        trajCommands.add(trajCommand(new Spline(pos.vec(), tangent.mult(v), end, Vec.dir(t).mult(v))));
    }
    private Command trajCommand(Path p) {
        Trajectory traj = new Trajectory(p, moveConstraints, vi, vf, pos.h);
        vi = vf;
        vf = 0;
        tf = traj.tf();
        pos = new Pose(p.pos(1), pos.h - p.vel(0).angle() + p.vel(1).angle());
        tangent = p.vel(1).normalize();
        return new FnCommand(t -> {
            traj.setTi(t);
            drive.setTrajectory(traj);}, t -> {}, (t, b) -> {}, t -> t > traj.tf(), drive);
    }
    private Command trajCommand(Path p, double hf) {
        Trajectory traj = new Trajectory(p, moveConstraints, turnConstraints, vi, vf, pos.h, hf);
        vi = vf;
        vf = 0;
        tf = traj.tf();
        pos = new Pose(p.pos(1), hf);
        tangent = p.vel(1).normalize();
        return new FnCommand(t -> {
            traj.setTi(t);
            drive.setTrajectory(traj);}, t -> {}, (t, b) -> {}, t -> t > traj.tf(), drive);
    }
    public Command build() {
        return new SeqCommand(trajCommands.toArray(new Command[0]));
    }
}