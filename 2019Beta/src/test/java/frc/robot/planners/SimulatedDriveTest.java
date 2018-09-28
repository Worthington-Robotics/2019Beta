package frc.robot.planners;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.trajectory.TimedView;
import frc.lib.trajectory.TrajectoryIterator;
import frc.robot.Constants;
import frc.robot.Kinematics;
import org.junit.jupiter.api.Test;

import java.util.Arrays;

public class SimulatedDriveTest {

    @Test
    public void simulateDriveOutput(){
        System.out.println("simulating a drive output");
        final DriveMotionPlanner motion_planner = new DriveMotionPlanner();
        motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.NONLINEAR_FEEDBACK);
        motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory
                (false, Arrays.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(120.0, -36.0), Rotation2d.identity()),
                        new Pose2d(new Translation2d(240.0, -36.0), Rotation2d.identity())),
                        null,
                        120.0, 120.0, 10.0))));
        final double dt = 0.01;
        double t = 0.0;
        Pose2d initial_error = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(3.5));
        Pose2d pose = motion_planner.setpoint().state().getPose().transformBy(initial_error);
        while (!motion_planner.isDone()) {
            DriveMotionPlanner.Output output = motion_planner.update(t, pose);
            Twist2d delta = Kinematics.forwardKinematics(
                    output.left_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0,
                    output.right_velocity * dt * Constants.kDriveWheelDiameterInches / 2.0);
            // Add some systemic error.
            delta = new Twist2d(delta.dx * 1.0, delta.dy * 1.0, delta.dtheta * 1.05);
            pose = pose.transformBy(Pose2d.exp(delta));
            t += dt;
            //                left pos    left velocity         right pos     right velocity        robot heading    error pose
            System.out.println(t + ",0.00," + output.left_velocity + ",0.00," + output.right_velocity + ",0.000," + motion_planner.mError.toCSV() + ",false,0.000,0.000,0.000,0.000," + motion_planner.setpoint().toCSV());
            //System.out.println(motion_planner.setpoint().toCSV() + "," + pose.toCSV());
        }
        System.out.println("final pose: " + pose);
    }

}
