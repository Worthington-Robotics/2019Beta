package frc.lib.statemachine;

import java.util.concurrent.ConcurrentLinkedQueue;

public class StateMacheneDiscriptor {
    private ConcurrentLinkedQueue<ActionGroup> Qedstates;

    public StateMacheneDiscriptor() {
        Qedstates = new ConcurrentLinkedQueue<>();
    }

    public void addSequntal(Action action, long timeout_ms) {
        Qedstates.add(new ActionGroup(action, timeout_ms));
    }

    public void addParallel(Action[] action, long timeout_ms) {
        Qedstates.add(new ActionGroup(action, timeout_ms));
    }

    public ConcurrentLinkedQueue getStates() {
        return Qedstates;
    }
}
