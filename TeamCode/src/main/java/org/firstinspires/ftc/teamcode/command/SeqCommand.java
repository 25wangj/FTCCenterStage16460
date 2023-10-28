package org.firstinspires.ftc.teamcode.command;

public class SeqCommand extends Command {
    public Command[] commands;
    public int index;
    public SeqCommand(Command... commands) {
        for (Command command : commands) {
            subsystems.addAll(command.getSubsystems());
        }
        this.commands = commands;
        index = 0;
    }
    @Override
    public void init() {
        commands[0].init();
    }
    @Override
    public void run() {
        if (index != commands.length - 1 && commands[index].done()) {
            commands[index].end(false);
            index++;
            commands[index].init();
        } else {
            commands[index].run();
        }
    }
    @Override
    public void end(boolean canceled) {
        commands[index].end(canceled);
    }
    @Override
    public boolean done() {
        return index == commands.length - 1 && commands[index].done();
    }
}
