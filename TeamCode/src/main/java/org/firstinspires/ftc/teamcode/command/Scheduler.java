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
    private HashSet<Command> added;
    private HashSet<Listener> listeners;
    private ElapsedTime clock;
    public Scheduler() {
        this(new ElapsedTime());
    }
    public Scheduler(ElapsedTime clock) {
        commands = new HashSet<>();
        subsystems = new HashMap<>();
        added = new HashSet<>();
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
                command.init();
                commands.add(command);
                for (Subsystem subsystem : command.getSubsystems()) {
                    subsystems.put(subsystem, command);
                }
            }
            for (Command command : commands) {
                if (command.done()) {
                    command.end(false);
                    for (Subsystem subsystem : command.getSubsystems()) {
                        subsystems.put(subsystem, null);
                    }
                    commands.remove(command);
                } else {
                    command.run();
                }
            }
            added.clear();
        }
    }
    public boolean schedule(Command command) {
        ArrayList<Command> toCancel = new ArrayList<>();
        if (commands.contains(command) || added.contains(command)) {
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
                command.end(true);
            }
        }
    }
    public void cancelAll() {
        for (Subsystem subsystem : subsystems.keySet()) {
            subsystems.put(subsystem, null);
        }
        for (Command command : commands) {
            command.end(true);
        }
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
