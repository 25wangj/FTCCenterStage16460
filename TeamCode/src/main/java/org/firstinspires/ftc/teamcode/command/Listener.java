package org.firstinspires.ftc.teamcode.command;
import org.firstinspires.ftc.teamcode.sensors.ConsecutiveDetector;
import org.firstinspires.ftc.teamcode.sensors.RisingEdgeDetector;
import java.util.function.BooleanSupplier;
public class Listener {
    private Command command;
    private BooleanSupplier condition;
    public static Listener risingEdge(BooleanSupplier condition, Command command) {
        return new Listener(new RisingEdgeDetector(condition), command);
    }
    public static Listener consecutive(BooleanSupplier condition, int num, Command command) {
        return new Listener(new ConsecutiveDetector(condition, num), command);
    }
    public Listener(BooleanSupplier condition, Command command) {
        this.condition = condition;
        this.command = command;
    }
    public boolean ready() {
        return condition.getAsBoolean();
    }
    public Command getCommand() {
        return command;
    }
}
