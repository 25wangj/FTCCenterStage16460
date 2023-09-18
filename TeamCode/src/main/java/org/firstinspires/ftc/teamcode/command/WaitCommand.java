package org.firstinspires.ftc.teamcode.command;
import com.qualcomm.robotcore.util.ElapsedTime;
public class WaitCommand extends Command {
    public ElapsedTime clock;
    public double start;
    public double seconds;
    public WaitCommand(double seconds) {
        clock = new ElapsedTime();
        start = 0;
        this.seconds = seconds;
    }
    public WaitCommand(ElapsedTime clock, double start, double seconds) {
        this.clock = clock;
        this.start = start;
        this.seconds = seconds;
    }
    @Override
    public void init() {}
    @Override
    public void run() {}
    @Override
    public void end(boolean canceled) {}
    @Override
    public boolean done() {
        return clock.seconds() > start + seconds;
    }
}
