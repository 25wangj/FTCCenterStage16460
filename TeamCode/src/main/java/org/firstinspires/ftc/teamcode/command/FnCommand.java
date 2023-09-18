package org.firstinspires.ftc.teamcode.command;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
public class FnCommand extends Command {
    private Runnable initFn;
    private Runnable runFn;
    private Consumer<Boolean> endFn;
    private BooleanSupplier doneFn;
    public static FnCommand once(Runnable initFn, Subsystem... systems) {
        return once(initFn, false, systems);
    }
    public static FnCommand once(Runnable initFn, boolean cancelable, Subsystem... systems) {
        return new FnCommand(initFn, () -> {}, b -> {}, () -> true, cancelable, systems);
    }
    public static FnCommand repeat(Runnable runFn, Subsystem... systems) {
        return repeat(runFn, false, systems);
    }
    public static FnCommand repeat(Runnable runFn, boolean cancelable, Subsystem... systems) {
        return new FnCommand(() -> {}, runFn, b -> {}, () -> false, cancelable, systems);
    }
    public static FnCommand until(BooleanSupplier doneFn) {
        return until(doneFn, false);
    }
    public static FnCommand until(BooleanSupplier doneFn, boolean cancelable) {
        return new FnCommand(() -> {}, () -> {}, b -> {}, doneFn, cancelable);
    }
    public FnCommand(Runnable initFn, Runnable runFn, Consumer<Boolean> endFn, BooleanSupplier doneFn, Subsystem... systems) {
        this(initFn, runFn, endFn, doneFn, false, systems);
    }
    public FnCommand(Runnable initFn, Runnable runFn, Consumer<Boolean> endFn, BooleanSupplier doneFn, boolean cancelable, Subsystem... systems) {
        subsystems.addAll(Arrays.asList(systems));
        this.cancelable = cancelable;
        this.initFn = initFn;
        this.runFn = runFn;
        this.endFn = endFn;
        this.doneFn = doneFn;
    }
    @Override
    public void init() {
        initFn.run();
    }
    @Override
    public void run() {
        runFn.run();
    }
    @Override
    public void end(boolean canceled) {
        endFn.accept(canceled);
    }
    @Override
    public boolean done() {
        return doneFn.getAsBoolean();
    }
}
