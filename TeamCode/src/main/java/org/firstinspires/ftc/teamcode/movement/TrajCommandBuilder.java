package org.firstinspires.ftc.teamcode.movement;
import android.util.Pair;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.Scheduler;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
public class TrajCommandBuilder {
    private Drivetrain drive;
    private Pose pos;
    private Vec tangent;
    private double vi;
    private double vf;
    private AsymConstraints moveConstraints;
    private AsymConstraints turnConstraints;
    private ArrayList<Trajectory> trajs;
    private ArrayList<Double> endTimes;
    private ArrayList<ArrayList<Command>> markers;
    private ArrayList<ArrayList<Double>> times;
    private ArrayList<Command> globalMarkers;
    private ArrayList<Pair<Double, Double>> globalTimes;
    public TrajCommandBuilder(Drivetrain drive, Pose pos) {
        this.drive = drive;
        this.pos = pos;
        this.tangent = Vec.dir(pos.h);
        this.moveConstraints = drive.moveConstraints;
        this.turnConstraints = drive.turnConstraints;
        trajs = new ArrayList<>();
        endTimes = new ArrayList<>();
        endTimes.add(0d);
        markers = new ArrayList<>();
        times = new ArrayList<>();
        globalMarkers = new ArrayList<>();
        globalTimes = new ArrayList<>();
    }
    public TrajCommandBuilder setTangent(double h) {
        tangent = Vec.dir(h);
        return this;
    }
    public TrajCommandBuilder setVf(double vf) {
        this.vf = vf;
        return this;
    }
    public TrajCommandBuilder setMoveConstraints(AsymConstraints moveConstraints) {
        this.moveConstraints = moveConstraints;
        return this;
    }
    public TrajCommandBuilder setTurnConstraints(AsymConstraints turnConstraints) {
        this.turnConstraints = turnConstraints;
        return this;
    }
    public TrajCommandBuilder resetConstraints() {
        moveConstraints = drive.moveConstraints;
        turnConstraints = drive.turnConstraints;
        return this;
    }
    public TrajCommandBuilder pause(double t) {
        pos = new Pose(pos.vec().combo(1, tangent, vi * t), pos.h);
        endTimes.add(endTimes.get(endTimes.size() - 1) + t);
        vf = 0;
        trajs.add(new WaitTrajectory(pos, tangent.mult(vi), t));
        markers.add(new ArrayList<>());
        return this;
    }
    public TrajCommandBuilder turn(double h) {
        if (vi != 0) {
            throw new IllegalArgumentException("Must be stationary to turn");
        }
        Trajectory traj = new TurnTrajectory(turnConstraints, pos, h);
        pos = new Pose(pos.vec(), h);
        endTimes.add(endTimes.get(endTimes.size() - 1) + traj.tf());
        vf = 0;
        tangent = Vec.dir(h);
        trajs.add(traj);
        markers.add(new ArrayList<>());
        return this;
    }
    public TrajCommandBuilder lineTo(Pose end) {
        addTraj(new Line(pos.vec(), end.vec()), end.h);
        return this;
    }
    public TrajCommandBuilder lineTo(Vec end) {
        addTraj(new Line(pos.vec(), end));
        return this;
    }
    public TrajCommandBuilder lineToX(double x, double h) {
        addTraj(Line.extendX(pos.vec(), tangent, x), h);
        return this;
    }
    public TrajCommandBuilder lineToX(double x) {
        addTraj(Line.extendX(pos.vec(), tangent, x));
        return this;
    }
    public TrajCommandBuilder lineToY(double y, double h) {
        addTraj(Line.extendY(pos.vec(), tangent, y), h);
        return this;
    }
    public TrajCommandBuilder lineToY(double y) {
        addTraj(Line.extendY(pos.vec(), tangent, y));
        return this;
    }
    public TrajCommandBuilder splineTo(Pose end, double t, double v) {
        addTraj(new Spline(pos.vec(), tangent.mult(v), end.vec(), Vec.dir(end.h).mult(v)), end.h);
        return this;
    }
    public TrajCommandBuilder splineTo(Vec end, double t, double v) {
        addTraj(new Spline(pos.vec(), tangent.mult(v), end, Vec.dir(t).mult(v)));
        return this;
    }
    public TrajCommandBuilder marker(Command command) {
        return marker(0, 0, command);
    }
    public TrajCommandBuilder marker(double scale, double offset, Command command) {
        if (trajs.isEmpty()) {
            globalMarkers.add(command);
            globalTimes.add(new Pair<>(scale, offset));
        } else {
            markers.get(markers.size() - 1).add(command);
            times.get(markers.size() - 1).add((endTimes.get(endTimes.size() - 1) - endTimes.get(endTimes.size() - 2))
                    * scale + offset);
        }
        return this;
    }
    private void addTraj(Path p) {
        Trajectory traj = new PathTrajectory(p, moveConstraints, vi, vf, pos.h);
        vi = vf;
        vf = 0;
        endTimes.add(endTimes.get(endTimes.size() - 1) + traj.tf());
        pos = new Pose(p.pos(1), pos.h - p.vel(0).angle() + p.vel(1).angle());
        tangent = p.vel(1).normalize();
        trajs.add(traj);
        markers.add(new ArrayList<>());
    }
    private void addTraj(Path p, double hf) {
        Trajectory traj = new PathTrajectory(p, moveConstraints, turnConstraints, vi, vf, pos.h, hf);
        vi = vf;
        vf = 0;
        endTimes.add(endTimes.get(endTimes.size() - 1) + traj.tf());
        pos = new Pose(p.pos(1), hf);
        tangent = p.vel(1).normalize();
        trajs.add(traj);
        markers.add(new ArrayList<>());
    }
    public Command build(Scheduler scheduler) {
        for (int i = 0; i < globalMarkers.size(); i++) {
            markers.get(0).add(globalMarkers.get(i));
            times.get(0).add(globalTimes.get(i).first * endTimes.get(endTimes.size() - 1) + globalTimes.get(i).second);
        }
        return new Command(drive) {
            private int index;
            private double ti;
            private HashMap<Command, Double> currMarkers;
            @Override
            public void init(double time) {
                index = 0;
                ti = time;
                trajs.get(0).setTi(ti);
                drive.setTrajectory(trajs.get(0));
                currMarkers = new HashMap<>();
                for (int i = 0; i < markers.get(0).size(); i++) {
                    currMarkers.put(markers.get(0).get(i), times.get(0).get(i) + ti);
                }
            }
            @Override
            public void run(double time) {
                if (index != trajs.size() - 1 && time > trajs.get(index).tf()) {
                    index++;
                    trajs.get(index).setTi(ti + endTimes.get(index));
                    drive.setTrajectory(trajs.get(index));
                    for (int i = 0; i < markers.get(index).size(); i++) {
                        currMarkers.put(markers.get(index).get(i), times.get(index).get(i) + endTimes.get(index) + ti);
                    }
                }
                for (Map.Entry<Command, Double> p : currMarkers.entrySet()) {
                    if (time > p.getValue()) {
                        scheduler.schedule(p.getKey());
                    }
                    currMarkers.remove(p.getKey());
                }
            }
            @Override
            public void end(double time, boolean canceled) {}
            @Override
            public boolean done(double time) {
                return time > ti + endTimes.get(endTimes.size() - 1);
            }
        };
    }
}