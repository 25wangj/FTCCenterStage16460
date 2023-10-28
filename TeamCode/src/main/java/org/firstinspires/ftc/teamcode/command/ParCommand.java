package org.firstinspires.ftc.teamcode.command;
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
    public void init() {
        for (Command command : commands) {
            command.init();
        }
    }
    @Override
    public void run() {
        for (int i = 0; i < commands.length; i++) {
            if (!dones[i]) {
                if (justDones[i]) {
                    commands[i].end(false);
                    justDones[i] = false;
                } else {
                    commands[i].run();
                }
            }
        }
    }
    @Override
    public void end(boolean canceled) {
        if (canceled) {
            for (int i = 0; i < commands.length; i++) {
                if (!dones[i]) {
                    commands[i].end(true);
                }
            }
        } else {
            for (int i = 0; i < commands.length; i++) {
                if (justDones[i]) {
                    commands[i].end(false);
                }
            }
        }
    }
    @Override
    public boolean done() {
        for (int i = 0; i < commands.length; i++) {
            if (!dones[i] && commands[i].done()) {
                justDones[i] = true;
                dones[i] = true;
                numDone++;
            }
        }
        return numDone == commands.length;
    }
}
