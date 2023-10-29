package org.firstinspires.ftc.teamcode.sensors;
import java.util.function.BooleanSupplier;
public class RisingEdgeDetector implements BooleanSupplier {
    private BooleanSupplier condition;
    private boolean last;
    public RisingEdgeDetector(BooleanSupplier condition) {
        this.condition = condition;
        last = false;
    }
    @Override
    public boolean getAsBoolean() {
        boolean temp = last;
        last = condition.getAsBoolean();
        return (last = condition.getAsBoolean()) && !temp;
    }
}
