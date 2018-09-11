package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {
    //Motors and Controllers
    public static final int DRIVE_FRONT_LEFT_ID = 1;
    public static final int DRIVE_MIDDLE_LEFT_ID = 2;
    public static final int DRIVE_BACK_LEFT_ID = 3;
    public static final int DRIVE_FRONT_RIGHT_ID = 4;
    public static final int DRIVE_MIDDLE_RIGHT_ID = 5;
    public static final int DRIVE_BACK_RIGHT_ID = 6;
    public static final int LIFT_LOWER_ID = 7;
    public static final int LIFT_UPPER_ID = 8;
    //Stuff that isn't mine
    //Update times / rates
    public static double LOOPER_DT = 0.01; //dt in seconds

    //MP Test mode values
    public static boolean ENABLE_MP_TEST_MODE = false; //enables motion profiling test across all modes
    public static double MP_TEST_SPEED = 152;

    //Pure pursuit related values
    public static final double kDriveWheelTrackWidthInches = 25.54;
    public static final double kDriveWheelDiameterInches = 3.92820959548 * 0.99;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!
    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
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

    //logging directories
    public static String DRIVE_PATH_1 = "/media/sda"; // top usb port
    public static String DRIVE_PATH_2 = "/media/sdb"; // bottom usb port

    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(new Joystick(0), 0.05, 1.0, 1.0, 1.0, 2);
    public static final HIDHelper.HIDConstants SECOND_STICK = new HIDHelper.HIDConstants(new Joystick(1), 0.05, 1.0, 1.0, 1.0, 2);

}

