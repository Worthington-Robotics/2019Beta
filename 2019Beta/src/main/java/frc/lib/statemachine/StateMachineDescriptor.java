package frc.lib.statemachine;

import java.util.concurrent.ConcurrentLinkedQueue;

public class StateMachineDescriptor {
    private ConcurrentLinkedQueue<ActionGroup> Qedstates;

    public StateMachineDescriptor() {
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
