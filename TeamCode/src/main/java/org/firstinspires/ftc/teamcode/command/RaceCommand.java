package org.firstinspires.ftc.teamcode.command;
public class RaceCommand extends Command {
    private Command[] commands;
    private boolean[] dones;
    private boolean done;
    public RaceCommand(Command... commands) {
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
        done = false;
    }
    @Override
    public void init() {
        for (Command command : commands) {
            command.init();
        }
    }
    @Override
    public void run() {
        for (Command command : commands) {
            command.run();
        }
    }
    @Override
    public void end(boolean canceled) {
        for (int i = 0; i < commands.length; i++) {
            if (dones[i]) {
                commands[i].end(canceled);
            }
        }
    }
    @Override
    public boolean done() {
        for (int i = 0; i < commands.length; i++) {
            if (commands[i].done()) {
                dones[i] = true;
                done = true;
            }
        }
        return done;
    }
}
