package org.firstinspires.ftc.teamcode.command;
import java.util.HashSet;
public abstract class Command {
    protected HashSet<Subsystem> subsystems = new HashSet<>();
    protected boolean cancelable = true;
    public abstract void init();
    public abstract void run();
    public abstract void end(boolean canceled);
    public HashSet<Subsystem> getSubsystems() {
        return subsystems;
    }
    public boolean isCancelable() {
        return cancelable;
    }
    public abstract boolean done();
}
