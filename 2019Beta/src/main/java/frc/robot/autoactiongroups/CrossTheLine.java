package frc.robot.autoactiongroups;

import frc.lib.geometry.State;
import frc.lib.statemachine.ActionGroup;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.lib.trajectory.Trajectory;
import frc.robot.actions.DriveTra;
import frc.robot.planners.TraGenerator;

public class CrossTheLine extends StateMachineDescriptor {
    public CrossTheLine() {
        addSequntal(new DriveTra(TraGenerator.getInstance().getTenFeet(), false), 30000 );


    }
}
