package frc.robot.autodescriptors;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.exampleAction;

public class ExampleDescriptor extends StateMachineDescriptor {

    public ExampleDescriptor(){
        //actions to execute in parallel
        addParallel(new Action[] {new exampleAction(),
                new exampleAction(), new exampleAction()}, 1000);
        //add single action in series
        addSequntal(new exampleAction(), 2000);
    }
}
