package org.firstinspires.ftc.teamcode.command;

public class SeqCommand extends Command {
    public Command[] commands;
    public int index;
    public boolean inInit;
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
        if (inInit) {
            commands[index].init();
            inInit = false;
        } else {
            if (index != commands.length - 1 && commands[index].done()) {
                index++;
                inInit = true;
            } else {
                commands[index].run();
            }
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
