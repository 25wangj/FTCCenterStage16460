package org.firstinspires.ftc.teamcode.command;
public class DeadlineCommand extends Command {
    private Command primary;
    private Command[] commands;
    boolean[] dones;
    public DeadlineCommand(Command primary, Command... commands) {
        subsystems.addAll(primary.getSubsystems());
        cancelable = cancelable && primary.isCancelable();
        for (Command command : commands) {
            for (Subsystem subsystem : command.getSubsystems()) {
                if (subsystems.contains(subsystem)) {
                    throw new IllegalArgumentException("Subsystem used twice");
                }
                subsystems.add(subsystem);
                cancelable = cancelable && command.isCancelable();
            }
        }
        this.primary = primary;
        this.commands = commands;
        dones = new boolean[commands.length];
    }
    @Override
    public void init() {
        primary.init();
        for (Command command : commands) {
            command.init();
        }
    }
    @Override
    public void run() {
        primary.run();
        for (int i = 0; i < commands.length; i++) {
            if (!dones[i]) {
                if (commands[i].done()) {
                    dones[i] = true;
                    commands[i].end(false);
                } else {
                    commands[i].run();
                }
            }
        }
    }
    @Override
    public void end(boolean canceled) {
        if (canceled) {
            primary.end(true);
            for (int i = 0; i < commands.length; i++) {
                if (!dones[i]) {
                    commands[i].end(true);
                }
            }
        }
        primary.end(false);
        for (Command command : commands) {
            if (command.done()) {
                command.end(false);
            }
        }
    }
    @Override
    public boolean done() {
        return primary.done();
    }
}
