package frc.lib.statemachine;

import edu.wpi.first.wpilibj.command.Command;

public abstract class Action {
    abstract void onStart();
    abstract void onLoop();
    abstract boolean isFinsihed();
    abstract void onStop();

    /**
     * converts an action to a wpilib command for buttons
     */
    public static Command toCommand(Action action){
        return new Command() {

            protected void initialize(){
                action.onStart();
            }

            protected void execute(){
                action.onLoop();
            }

            protected boolean isFinished() {
                return action.isFinsihed();
            }

            protected void end(){
                action.onStop();
            }
        };
    }
}
