package org.firstinspires.ftc.teamcode.command;
import java.util.HashSet;
public class ParCommand extends Command {
    public Command[] commands;
    public boolean[] dones;
    public boolean[] justDones;
    public int numDone;
    public ParCommand(Command... commands) {
        for (Command command : commands) {
            for (Subsystem subsystem : command.getSubsystems()) {
                if (subsystems.contains(subsystem)) {
                    throw new IllegalArgumentException("Subsystem used twice");
                }
                subsystems.add(subsystem);
            }
        }
        this.commands = commands;
        dones = new boolean[commands.length];
        justDones = new boolean[commands.length];
        numDone = 0;
    }
    @Override
    public void init(double time) {
        for (Command command : commands) {
            command.init(time);
        }
    }
    @Override
    public void run(double time) {
        for (int i = 0; i < commands.length; i++) {
            if (!dones[i]) {
                if (justDones[i]) {
                    commands[i].end(time, false);
                    justDones[i] = false;
                } else {
                    commands[i].run(time);
                }
            }
        }
    }
    @Override
    public void end(double time, boolean canceled) {
        if (canceled) {
            for (int i = 0; i < commands.length; i++) {
                if (!dones[i]) {
                    commands[i].end(time, true);
                }
            }
        } else {
            for (int i = 0; i < commands.length; i++) {
                if (justDones[i]) {
                    commands[i].end(time, false);
                }
            }
        }
    }
    @Override
    public boolean done(double time) {
        for (int i = 0; i < commands.length; i++) {
            if (!dones[i] && commands[i].done(time)) {
                justDones[i] = true;
                dones[i] = true;
                numDone++;
            }
        }
        return numDone == commands.length;
    }
}
