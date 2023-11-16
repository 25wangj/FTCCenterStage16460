package org.firstinspires.ftc.teamcode.movement;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.Scheduler;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
public class TrajCommandBuilder {
    private Drivetrain drive;
    private Pose pos;
    private Vec tangent;
    private double vi;
    private double vf;
    private AsymConstraints moveConstraints;
    private AsymConstraints turnConstraints;
    private List<Trajectory> trajs = new ArrayList<>();
    private List<Double> endTimes = new ArrayList<>(Arrays.asList(0d));
    private List<List<Command>> markers = new ArrayList<>();
    private List<List<Double>> times = new ArrayList<>();
    private List<Command> globalMarkers = new ArrayList<>();
    private List<Double> globalTimes = new ArrayList<>();
    public TrajCommandBuilder(Drivetrain drive, Pose pos) {
        this.drive = drive;
        this.pos = pos;
        this.tangent = Vec.dir(pos.h);
        moveConstraints = drive.getMoveConstraints();
        turnConstraints = drive.getTurnConstraints();
    }
    public TrajCommandBuilder(Drivetrain drive, Pose pos, Vec vel) {
        this.drive = drive;
        this.pos = pos;
        this.tangent = vel.normalize();
        vi = vel.norm();
        moveConstraints = drive.getMoveConstraints();
        turnConstraints = drive.getTurnConstraints();
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
        moveConstraints = drive.getMoveConstraints();
        turnConstraints = drive.getTurnConstraints();
        return this;
    }
    public TrajCommandBuilder pause(double t) {
        pos = new Pose(pos.vec().combo(1, tangent, vi * t), pos.h);
        vf = 0;
        endTimes.add(endTimes.get(endTimes.size() - 1) + t);
        trajs.add(new WaitTrajectory(pos, tangent.mult(vi), t));
        markers.add(new ArrayList<>());
        times.add(new ArrayList<>());
        return this;
    }
    public TrajCommandBuilder turn(double h) {
        if (vi != 0) {
            throw new IllegalArgumentException("Must be stationary to turn");
        }
        Trajectory traj = new TurnTrajectory(turnConstraints, pos, h);
        pos = new Pose(pos.vec(), h);
        tangent = Vec.dir(pos.h);
        vf = 0;
        endTimes.add(endTimes.get(endTimes.size() - 1) + traj.tf());
        trajs.add(traj);
        markers.add(new ArrayList<>());
        times.add(new ArrayList<>());
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
    public TrajCommandBuilder splineTo(Pose end, double t) {
        return splineTo(end, t, end.vec().combo(1, pos.vec(), -1).norm());
    }
    public TrajCommandBuilder splineTo(Vec end, double t) {
        return splineTo(end, t, end.combo(1, pos.vec(), -1).norm());
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
            globalTimes.add(scale);
            globalTimes.add(offset);
        } else {
            markers.get(markers.size() - 1).add(command);
            times.get(markers.size() - 1).add(endTimes.get(endTimes.size() - 2) +
                    (endTimes.get(endTimes.size() - 1) - endTimes.get(endTimes.size() - 2)) * scale + offset);
        }
        return this;
    }
    private void addTraj(Path p) {
        Trajectory traj = new PathTrajectory(p, moveConstraints, vi, vf, pos.h);
        pos = new Pose(p.state(1).pos, pos.h - p.state(0).vel.angle() + p.state(1).vel.angle());
        tangent = vf == 0 ? Vec.dir(pos.h) : p.state(1).vel.normalize();
        vi = vf;
        vf = 0;
        endTimes.add(endTimes.get(endTimes.size() - 1) + traj.tf());
        trajs.add(traj);
        markers.add(new ArrayList<>());
        times.add(new ArrayList<>());
    }
    private void addTraj(Path p, double hf) {
        Trajectory traj = new PathTrajectory(p, moveConstraints, turnConstraints, vi, vf, pos.h, hf);
        pos = new Pose(p.state(1).pos, hf);
        tangent = vf == 0 ? Vec.dir(pos.h) : p.state(1).vel.normalize();
        vi = vf;
        vf = 0;
        endTimes.add(endTimes.get(endTimes.size() - 1) + traj.tf());
        trajs.add(traj);
        markers.add(new ArrayList<>());
        times.add(new ArrayList<>());
    }
    public Command build(Scheduler scheduler) {
        for (int i = 0; i < globalMarkers.size(); i++) {
            markers.get(0).add(globalMarkers.get(i));
            times.get(0).add(globalTimes.get(2 * i) * endTimes.get(endTimes.size() - 1) + globalTimes.get(2 * i + 1));
        }
        System.out.println(endTimes);
        return new Command(drive) {
            private int index;
            private double ti;
            private Map<Command, Double> currMarkers;
            private ArrayList<Command> removed = new ArrayList<>();
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
                if (index != trajs.size() - 1 && time > ti + endTimes.get(index + 1)) {
                    index++;
                    trajs.get(index).setTi(endTimes.get(index) + ti);
                    drive.setTrajectory(trajs.get(index));
                    for (int i = 0; i < markers.get(index).size(); i++) {
                        currMarkers.put(markers.get(index).get(i), times.get(index).get(i) + ti);
                    }
                }
                for (Map.Entry<Command, Double> p : currMarkers.entrySet()) {
                    if (time > p.getValue()) {
                        scheduler.schedule(p.getKey());
                        removed.add(p.getKey());
                    }
                }
                for (Command command : removed) {
                    currMarkers.remove(command);
                }
                removed.clear();
            }
            @Override
            public void end(double time, boolean canceled) {
                for (Map.Entry<Command, Double> p : currMarkers.entrySet()) {
                    if (time > p.getValue()) {
                        scheduler.schedule(p.getKey());
                    }
                }
            }
            @Override
            public boolean done(double time) {
                return time > ti + endTimes.get(endTimes.size() - 1);
            }
        };
    }
}