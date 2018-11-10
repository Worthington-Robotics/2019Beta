package frc.robot.planners;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TraGenerator {
    private final static TraGenerator m_instance = new TraGenerator();
    private final DriveMotionPlanner DMP;

    private TraGenerator() {
        DMP = new DriveMotionPlanner();
    }

    public static TraGenerator getInstance() {
        return m_instance;
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return DMP.generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }
    public Trajectory<TimedState<Pose2dWithCurvature>> getTenFeet()
    {
        List<Pose2d> Points = new ArrayList<>();
        Points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        Points.add(new Pose2d(60, 60, Rotation2d.fromDegrees(90)));
        Points.add(new Pose2d(0,120, Rotation2d.fromDegrees(180)));
        Points.add(new Pose2d(-60,60, Rotation2d.fromDegrees(270)));
        Points.add(new Pose2d(12,-12, Rotation2d.fromDegrees(0)));


        return generateTrajectory(false, Points, Arrays.asList(new CentripetalAccelerationConstraint(100.0)), 120.0, 80.0, 8.0);
    }
}
