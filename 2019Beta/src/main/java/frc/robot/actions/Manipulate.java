package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Forks;

public class Manipulate extends Action {
    private ShotPower speed;

    public Manipulate(ShotPower Speed) {
        speed = Speed;
    }

    public void onStart() {
        Forks.getInstance().setShotPower(speed.shotpower);
    }

    public void onLoop() {
    }

    public boolean isFinished() {
        return false;
    }

    public void onStop() {
        Forks.getInstance().setShotPower(ShotPower.Stop.shotpower);
    }

    public enum ShotPower {
        Shoot(Constants.SHOOT_POWER),
        Rollout(Constants.ROLLOUT_POWER),
        Drop(Constants.DROP_POWER),
        PickUp(Constants.PICKUP_POWER),
        SlowUp(Constants.SLOWUP_POWER),
        Stop(Constants.STOP_POWER);

        private double shotpower;

        ShotPower(double power) {
            shotpower = power;
        }
    }

}
