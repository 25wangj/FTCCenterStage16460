package org.firstinspires.ftc.teamcode.command;
import android.util.Pair;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.function.Function;
public class StateMachineBuilder<T extends Enum<T>> {
    private HashSet<T> states;
    private HashMap<Pair<T, T>, Function<double[], Command>> transitions;
    private Scheduler scheduler;
    public StateMachineBuilder(CommandOpMode opMode) {
        scheduler = opMode.scheduler;
        states = new HashSet<>();
        transitions = new HashMap<>();
    }
    public StateMachineBuilder<T> addState(T... state) {
        states.addAll(Arrays.asList(state));
        return this;
    }
    public StateMachineBuilder<T> addTransition(T start, T end, Command command) {
        return addTransition(start, end, d -> command);
    }
    public StateMachineBuilder<T> addTransition(T start, T end, Function<double[], Command> fn) {
        if (!states.contains(start) || !states.contains(end)) {
            throw new IllegalArgumentException("State does not exist");
        }
        transitions.put(new Pair<>(start, end), fn);
        return this;
    }
    public StateMachine<T> build(T state) {
        return new StateMachine<>(scheduler, state, states, transitions);
    }
}