package org.firstinspires.ftc.teamcode.command;
public class RaceCommand extends Command {
    private Command[] commands;
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
        for (Command command : commands) {
            command.end(canceled);
        }
    }
    @Override
    public boolean done() {
        for (Command command : commands) {
            if (command.done()) {
                return true;
            }
        }
        return false;
    }
}
