package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {
    public static final boolean TRUE = true;
    //Motors and Controllers
    public static final int DRIVE_FRONT_LEFT_ID = 1;
    public static final int DRIVE_MIDDLE_LEFT_ID = 2;
    public static final int DRIVE_BACK_LEFT_ID = 3;
    public static final int DRIVE_FRONT_RIGHT_ID = 4;
    public static final int DRIVE_MIDDLE_RIGHT_ID = 5;
    public static final int DRIVE_BACK_RIGHT_ID = 6;
    public static final int LIFT_LOWER_ID = 7;
    public static final int LIFT_UPPER_ID = 8;
    public static final double RKP = 0.2051;
    public static final double RKI = 0;
    public static final double RKD = 0.3;
    public static final double RKF = 0.07185;
    public static final double LKP = 0.2051;
    public static final double LKI = 0;
    public static final double LKD = 0.3;
    public static final double LKF = 0.07185;

    //
    public static final int FORKSUD_ID = 4;
    public static final int RIGHT_SHOOTER_ID = 3;
    public static final int LEFT_SHOOTER_ID = 2;
    //
    public static final int LOWER_LIFT_ENCODER_A = 5;
    public static final int LOWER_LIFT_ENCODER_B = 6;
    //
    public static final int TRANS_LOW_ID = 0;
    public static final int TRANS_HIGH_ID = 1;

    //Pure pursuit related values
    public static final double kDriveWheelTrackWidthInches = 23.54;
    public static final double kDriveWheelDiameterInches = 6.5;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!
    public static final double kRobotLinearInertia = 66.5;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kRobotMaxVelocity = 120.0; // TODO tune & find units
    public static final double kRobotMaxAccel = 120.0; // TODO tune & find units
    public static final double kRobotMaxVoltage = 10.0; // V TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.135;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2
    public static final double DRIVE_ENCODER_PPR = 4096.0; //encoder counts per revolution
    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    //Numerical Constants
    public static final double SHOOT_POWER = 1;
    public static final double ROLLOUT_POWER = .9;
    public static final double DROP_POWER = .65;
    public static final double PICKUP_POWER = -.75;
    public static final double SLOWUP_POWER = -.65;
    public static final double STOP_POWER = 0;
    public static final double TICKS_TO_INCHES = 1625;
    public static final double ROTATIONS_TO_INCHES = TICKS_TO_INCHES/DRIVE_ENCODER_PPR;
    public static final String ROBOT_NAME = "Whatever_you_want";
    public static final double LOGGING_UPDATE_RATE = .02;


    //Stuff that isn't mine
    //Update times / rates
    public static double LOOPER_DT = 0.01; //dt in seconds

    //MP Test mode values
    public static boolean ENABLE_MP_TEST_MODE = TRUE; //enables motion profiling test across all modes
    public static double MP_TEST_SPEED = 12;



    //logging directories
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick SECOND = new Joystick(1);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.15, 1.0, 1.0, -0.45, 2);
    public static final HIDHelper.HIDConstants SECOND_STICK = new HIDHelper.HIDConstants(SECOND, 0.05, 1.0, 1.0, 1.0, 2);

    public static boolean isCompBot = TRUE;
}


