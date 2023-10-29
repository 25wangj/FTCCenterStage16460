package org.firstinspires.ftc.teamcode.command;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
public class Scheduler {
    private HashSet<Command> commands;
    private HashMap<Subsystem, Command> subsystems;
    private ArrayList<Command> added;
    private ArrayList<Command> removed;
    private HashSet<Listener> listeners;
    private ElapsedTime clock;
    public Scheduler() {
        this(new ElapsedTime());
    }
    public Scheduler(ElapsedTime clock) {
        commands = new HashSet<>();
        subsystems = new HashMap<>();
        added = new ArrayList<>();
        removed = new ArrayList<>();
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
            for (Command command : added) {
                command.init(time);
                commands.add(command);
                for (Subsystem subsystem : command.getSubsystems()) {
                    subsystems.put(subsystem, command);
                }
            }
            added.clear();
            for (Command command : removed) {
                command.end(time, true);
            }
            removed.clear();
            for (Command command : commands) {
                if (command.done(time)) {
                    removed.add(command);
                } else {
                    command.run(time);
                }
            }
            for (Command command : removed) {
                command.end(time, false);
                for (Subsystem subsystem : command.getSubsystems()) {
                    subsystems.put(subsystem, null);
                }
                commands.remove(command);
            }
            removed.clear();
        }
    }
    public boolean schedule(Command command) {
        ArrayList<Command> toCancel = new ArrayList<>();
        if (commands.contains(command)) {
            return false;
        }
        for (Subsystem subsystem : command.getSubsystems()) {
            if (!subsystems.containsKey(subsystem)) {
                throw new IllegalArgumentException("Command uses unregistered subsystem");
            } else if (subsystems.get(subsystem) != null && !subsystems.get(subsystem).isCancelable()) {
                return false;
            }
            toCancel.add(subsystems.get(subsystem));
        }
        added.add(command);
        cancel(toCancel.toArray(new Command[0]));
        return true;
    }
    public void cancel(Command... toCancel) {
        for (Command command : toCancel) {
            if (commands.contains(command)) {
                for (Subsystem subsystem : command.getSubsystems()) {
                    subsystems.put(subsystem, null);
                }
                commands.remove(command);
                removed.add(command);
            }
        }
    }
    public void cancelAll() {
        for (Subsystem subsystem : subsystems.keySet()) {
            subsystems.put(subsystem, null);
        }
        removed.addAll(commands);
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
