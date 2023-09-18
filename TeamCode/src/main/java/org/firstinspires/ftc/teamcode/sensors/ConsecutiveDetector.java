package org.firstinspires.ftc.teamcode.sensors;
import java.util.function.BooleanSupplier;
public class ConsecutiveDetector implements BooleanSupplier {
    private BooleanSupplier condition;
    private int num;
    private int total;
    public ConsecutiveDetector(BooleanSupplier condition, int num) {
        this.condition = condition;
        this.num = num;
        total = 0;
    }
    @Override
    public boolean getAsBoolean() {
        total = condition.getAsBoolean() ? total + 1 : 0;
        return total >= num;
    }
}
