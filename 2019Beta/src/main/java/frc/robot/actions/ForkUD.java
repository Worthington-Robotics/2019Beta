package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Forks;

public class ForkUD extends Action {
    private double speed = 1;

    public ForkUD(boolean Up) {
        if (!Up) {
            speed = -speed;
        }
    }

    public void onStart() {
        Forks.getInstance().setUDPower(speed);
    }

    public void onLoop() {
    }

    public boolean isFinished() {
        return false;
    }

    public void onStop() {
        Forks.getInstance().setUDPower(0);
    }
}

