package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.lib.util.ReflectingCSVWriter;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.planners.DriveMotionPlanner;

import java.util.List;

public class Drive extends Subsystem {

    //used internally for data

    //construct one and only 1 instance of this class
    private static Drive m_DriveInstance = new Drive();
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private DriveMotionPlanner mMotionPlanner;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter;
    private PeriodicIO periodicIO;
    private Rotation2d mGyroOffset;
    private boolean mOverrideTrajectory = false;
    private WPI_TalonSRX driveFrontLeft;
    private WPI_TalonSRX driveMiddleLeft;
    private WPI_TalonSRX driveBackLeft;
    private WPI_TalonSRX driveFrontRight;
    private WPI_TalonSRX driveMiddleRight;
    private WPI_TalonSRX driveBackRight;
    private double[] operatorInput = {0, 0, 0}; //last input set from joystick update
    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                startLogging();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                if (Constants.ENABLE_MP_TEST_MODE) mDriveControlState = DriveControlState.PROFILING_TEST;
                switch (mDriveControlState) {
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;

                    case PROFILING_TEST:
                        if (DriverStation.getInstance().isTest()) {
                            //driveTank(Constants.MP_TEST_SPEED, Constants.MP_TEST_SPEED);
                        }
                        break;

                    case OPEN_LOOP:
                        if (DriverStation.getInstance().isOperatorControl())
                            operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);

                        else operatorInput = new double[]{0, 0, 0};
                        SmartDashboard.putNumberArray("stick", operatorInput);
                        setOpenLoop(arcadeDrive(operatorInput[1], operatorInput[2]));
                        break;

                    default:
                        System.out.println("You fool, unexpected control state");
                }


            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    private Drive() {
        mMotionPlanner = new DriveMotionPlanner();
        reset();
        mCSVWriter = new ReflectingCSVWriter<PeriodicIO>("", PeriodicIO.class);
            driveFrontLeft = new WPI_TalonSRX(Constants.DRIVE_FRONT_LEFT_ID);
          driveMiddleLeft = new WPI_TalonSRX(Constants.DRIVE_MIDDLE_LEFT_ID);
              driveBackLeft = new WPI_TalonSRX(Constants.DRIVE_BACK_LEFT_ID);
          driveFrontRight = new WPI_TalonSRX(Constants.DRIVE_FRONT_RIGHT_ID);
        driveMiddleRight = new WPI_TalonSRX(Constants.DRIVE_MIDDLE_RIGHT_ID);
            driveBackRight = new WPI_TalonSRX(Constants.DRIVE_BACK_RIGHT_ID);
    }

    public static Drive getInstance() {
        return m_DriveInstance;
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    public synchronized Rotation2d getHeading() {
        return periodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(0).inverse()); //TODO replace zero with gyro source
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        periodicIO.gyro_heading = heading;
    }

    public double getLeftEncoderRotations() {
        return periodicIO.left_pos_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return periodicIO.right_pos_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getLeftVelocityNativeUnits() {
        return periodicIO.left_velocity_ticks_per_100ms;
    }

    public double getRightVelocityNativeUnits() {
        return periodicIO.right_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / Constants.DRIVE_ENCODER_PPR);
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / Constants.DRIVE_ENCODER_PPR);
    }

    public void setOperatorInput(double[] input) {
        operatorInput = input;
    }

    public void reset() {
        mOverrideTrajectory = false;
        mMotionPlanner.reset();
        periodicIO = new PeriodicIO();
        //TODO add reset with sensor impl

    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            periodicIO.error = mMotionPlanner.error();
            periodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(new DriveSignal(inchesPerSecondToRpm(output.linear_velocity), Math.toDegrees(output.angular_position)));
                //TODO will require additional math to convert from heading to steering angle

            } else {
                setVelocity(DriveSignal.BRAKE);
                mDriveControlState = DriveControlState.OPEN_LOOP;
                mMotionPlanner.reset();

            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    private void configTele() {
        reset();
        driveFrontLeft.set(ControlMode.PercentOutput, 0);
        driveMiddleLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
        driveBackLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
        driveFrontRight.set(ControlMode.PercentOutput, 0);
        driveMiddleRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
        driveBackRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
    }

    private DriveSignal arcadeDrive(double xSpeed, double zRotation) {
        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }
    return new DriveSignal(rightMotorOutput, leftMotorOutput);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            System.out.println("Switching to open loop");
            configTele();
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal/*, DriveSignal feedforward*/) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            //TODO configure motor control for velocity

            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        periodicIO.left_demand = signal.getLeft(); //TODO convert to native units
        periodicIO.right_demand = signal.getRight(); //TODO convert to native units
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,
            double end_vel,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = periodicIO.left_pos_ticks;
        double prevRightTicks = periodicIO.right_pos_ticks;
        periodicIO.left_pos_ticks = 0; //TODO Add data source replacing zero
        periodicIO.right_pos_ticks = 0; //TODO Add data source replacing zero
        periodicIO.left_velocity_ticks_per_100ms = 0; //TODO Add data source replacing zero
        periodicIO.right_velocity_ticks_per_100ms = 0; //TODO Add data source replacing zero
        periodicIO.gyro_heading = Rotation2d.fromDegrees(0).rotateBy(mGyroOffset); //TODO Add data source replacing zero

        double deltaLeftTicks = ((periodicIO.left_pos_ticks - prevLeftTicks) / 4096.0) * Math.PI;
        if (deltaLeftTicks > 0.0) {
            periodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        } else {
            periodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        }


        double deltaRightTicks = ((periodicIO.right_pos_ticks - prevRightTicks) / 4096.0) * Math.PI;
        if (deltaRightTicks > 0.0) {
            periodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        } else {
            periodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        }

        if (mCSVWriter != null) {
            mCSVWriter.add(periodicIO);
        }

        // System.out.println("control state: " + mDriveControlState + ", left: " + periodicIO.linear_demand + ", right: " + periodicIO.angular_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            //TODO write open loop outputs
            driveFrontLeft.set(ControlMode.PercentOutput, periodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            driveMiddleLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
            driveBackLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
            driveFrontRight.set(ControlMode.PercentOutput, periodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
            driveMiddleRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
            driveBackRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
        } else {
            //TODO write velocity control mode outputs
            driveFrontLeft.set(ControlMode.PercentOutput, periodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            driveMiddleLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
            driveBackLeft.set(ControlMode.Follower, driveFrontLeft.getDeviceID());
            driveFrontRight.set(ControlMode.PercentOutput, periodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
            driveMiddleRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
            driveBackRight.set(ControlMode.Follower, driveFrontRight.getDeviceID());
            /*mLeftMaster.set(ControlMode.Velocity, periodicIO.linear_demand, DemandType.ArbitraryFeedForward,
            //       periodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * periodicIO.left_accel / 1023.0);
            //mRightMaster.set(ControlMode.Velocity, periodicIO.angular_demand, DemandType.ArbitraryFeedForward,
            //       periodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * periodicIO.right_accel / 1023.0);*/
            }
    }


    public void outputTelemetry() {

        if (mCSVWriter != null) {
            mCSVWriter.add(periodicIO);
            mCSVWriter.write();
        }
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    @Override
    public void stop() {

    }

    enum DriveControlState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        PROFILING_TEST;

        @Override
        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public int left_pos_ticks;
        public double left_dist;
        public int left_velocity_ticks_per_100ms;
        public double right_pos_ticks;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double right_distance;
        public double left_distance;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }

}
