package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Forks;

public class exampleAction extends Action {

    public exampleAction(){
    }

    public void onStart(){
        Forks.getInstance().setShotPower(1);
    }

    public void onLoop(){

    }

    public boolean isFinished(){
        return false;
    }

    public void onStop(){
        Forks.getInstance().setShotPower(0.0);
    }

}
