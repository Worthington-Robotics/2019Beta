package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;

public class ActionGroup {
    private LinkedList<Action> group;
    private double t_Start, t_Timeout;

    ActionGroup(Action[] actions, long timeout_ms) {
        t_Timeout = (double) timeout_ms / 1000.000;
        group = new LinkedList<>();
        for (Action action : actions) {
            group.add(action);
        }

    }

    ActionGroup(Action action, long timeout_ms) {
        t_Timeout = (double) timeout_ms / 1000.00000000;
        group = new LinkedList<>();
        group.add(action);
    }

    public void onStart() {
        t_Start = Timer.getFPGATimestamp();
        group.forEach(action -> action.onStart());
    }

    public void onLoop() {
        group.forEach(action -> action.onLoop());
    }

    public boolean isFinnished() {
        if (t_Start + t_Timeout <= Timer.getFPGATimestamp()) return true;
        boolean temp = true;
        for (Action action : group) {
            temp &= action.isFinished();
        }
        return temp;
    }

    public void onStop() {
        group.forEach(action -> action.onStop());
    }

}

