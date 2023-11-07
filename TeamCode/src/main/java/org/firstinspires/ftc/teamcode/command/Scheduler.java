package org.firstinspires.ftc.teamcode.command;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Queue;
import java.util.Set;
public class Scheduler {
    private HashSet<Command> commands;
    private HashMap<Subsystem, Command> subsystems;
    private Queue<Command> added;
    private Queue<Command> canceled;
    private ArrayList<Command> finished;
    private HashSet<Listener> listeners;
    private ElapsedTime clock;
    public Scheduler() {
        this(new ElapsedTime());
    }
    public Scheduler(ElapsedTime clock) {
        commands = new HashSet<>();
        subsystems = new HashMap<>();
        added = new ArrayDeque<>();
        canceled = new ArrayDeque<>();
        finished = new ArrayList<>();
        listeners = new HashSet<>();
        this.clock = clock;
    }
    public void run(boolean active) {
        double time = clock.seconds();
        for (Subsystem subsystem : subsystems.keySet()) {
            subsystem.update(time, active);
        }
        if (active) {
            for (Listener listener : listeners) {
                if (listener.ready()) {
                    schedule(listener.getCommand());
                }
            }
            for (int i = canceled.size(); i > 0; i--) {
                Command command = canceled.poll();
                for (Subsystem subsystem : command.getSubsystems()) {
                    subsystems.put(subsystem, null);
                }
                commands.remove(command);
                command.end(time, true);
            }
            for (int i = added.size(); i > 0; i--) {
                Command command = added.poll();
                command.init(time);
                commands.add(command);
                for (Subsystem subsystem : command.getSubsystems()) {
                    subsystems.put(subsystem, command);
                }
            }
            for (Command command : commands) {
                if (command.done(time)) {
                    finished.add(command);
                } else {
                    command.run(time);
                }
            }
            for (Command command : finished) {
                for (Subsystem subsystem : command.getSubsystems()) {
                    subsystems.put(subsystem, null);
                }
                commands.remove(command);
                command.end(time, false);
            }
        }
    }
    public boolean schedule(Command command) {
        HashSet<Command> toCancel = new HashSet<>();
        if (command == null || commands.contains(command)) {
            return false;
        }
        for (Subsystem subsystem : command.getSubsystems()) {
            if (!subsystems.containsKey(subsystem)) {
                throw new IllegalArgumentException("Unregistered subsystem");
            } else if (subsystems.get(subsystem) != null) {
                if (subsystems.get(subsystem).cancelable) {
                    toCancel.add(subsystems.get(subsystem));
                } else {
                    return false;
                }
            }
        }
        added.offer(command);
        cancel(toCancel.toArray(new Command[0]));
        return true;
    }
    public void cancel(Command... toCancel) {
        for (Command command : toCancel) {
            if (commands.contains(command)) {
                canceled.offer(command);
            }
        }
    }
    public void cancelAll() {
        for (Subsystem subsystem : subsystems.keySet()) {
            subsystems.put(subsystem, null);
        }
        canceled.addAll(commands);
        commands.clear();
        added.clear();
    }
    public void register(Subsystem... toAdd) {
        for (Subsystem subsystem : toAdd) {
            subsystems.put(subsystem, null);
        }
    }
    public void unregister(Subsystem... toRemove) {
        for (Subsystem subsystem : toRemove) {
            if (subsystems.get(subsystem) != null) {
                throw new IllegalArgumentException("Subsystem in use by a command");
            }
            subsystems.remove(subsystem);
        }
    }
    public void unregisterAll() {
        for (Subsystem subsystem : subsystems.keySet()) {
            if (subsystems.get(subsystem) != null) {
                throw new IllegalArgumentException("Subsystem in use by a command");
            }
        }
        subsystems.clear();
    }
    public void addListener(Listener... toAdd) {
        listeners.addAll(Arrays.asList(toAdd));
    }
    public void removeListener(Listener... toRemove) {
        listeners.removeAll(Arrays.asList(toRemove));
    }
    public void clearListeners() {
        listeners.clear();
    }
    public Set<Command> getCommands() {
        return commands;
    }
    public Set<Subsystem> getSubsystems() {
        return subsystems.keySet();
    }
    public Set<Listener> getListeners() {
        return listeners;
    }
}
