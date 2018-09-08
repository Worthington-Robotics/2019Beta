package frc.robot.subsystems;

import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Kinematics;
import frc.robot.RobotState;


public class RobotStateEstimator extends Subsystem {

    static RobotStateEstimator instance_ = new RobotStateEstimator();
    private RobotState robot_state_ = RobotState.getInstance();
    private Drive drive_ = Drive.getInstance();
    private double left_encoder_prev_distance = 0.0;
    private double right_encoder_prev_distance = 0.0;
    private double back_encoder_prev_distance = 0.0;

    RobotStateEstimator() {
    }

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    /**
     * Unused. Call RobotState.outputTelemetry()
     */
    @Override
    public void outputTelemetry() {

    }

    /**
     * Unused
     */
    @Override
    public void stop() {

    }

    /**
     * Unused. Call RobotState.reset()
     */
    @Override
    public void reset() {

    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new EnabledLoop());
    }

    private class EnabledLoop implements Loop {
        @Override
        public synchronized void onStart(double timestamp) {
            left_encoder_prev_distance = drive_.getLeftEncoderDistance();
            right_encoder_prev_distance = drive_.getRightEncoderDistance();
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            final double leftDistance = drive_.getLeftEncoderDistance();
            final double rightDistance = drive_.getRightEncoderDistance();
            final double deltaLeft = leftDistance - left_encoder_prev_distance;
            final double deltaRight = leftDistance - right_encoder_prev_distance;
            final Rotation2d gyro_angle = drive_.getHeading();
            final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(deltaLeft, deltaRight, gyro_angle);
            final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftLinearVelocity(),  drive_.getRightLinearVelocity());
            robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity); //ignore predicted velocity as it goes mostly unused
            left_encoder_prev_distance = leftDistance;
            right_encoder_prev_distance = rightDistance;
        }

        @Override
        public void onStop(double timestamp) {
            // no-op
        }
    }
}