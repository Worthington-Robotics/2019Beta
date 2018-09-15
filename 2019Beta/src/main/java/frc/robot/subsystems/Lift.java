package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Lift extends Subsystem {
    //The one instance of Lift
    private static Lift m_LiftInstance = new Lift();
    private double[] operatorInput = {0, 0, 0};
    private PeriodicIO periodic;
    private WPI_TalonSRX liftLower;
    private WPI_TalonSRX liftUpper;
    private Encoder lowerLift;
    private final Loop mLoop = new Loop() {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {

        }

        public void onStop(double timestamp) {
            liftLower.set(0);
            liftUpper.set(0);
        }
    };

    public Lift() {
        periodic = new PeriodicIO();
        liftLower = new WPI_TalonSRX(Constants.LIFT_LOWER_ID);
        liftUpper = new WPI_TalonSRX(Constants.LIFT_UPPER_ID);
        lowerLift = new Encoder(Constants.LOWER_LIFT_ENCODER_A, Constants.LOWER_LIFT_ENCODER_B);
    }

    public static Lift getInstance() {
        return m_LiftInstance;
    }

    public void writeToLog() {
    }

    public void setOperatorInput(double[] input) {
        operatorInput = input;
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {
        setOperatorInput(HIDHelper.getAdjStick(Constants.SECOND_STICK));
        periodic.trigger = Constants.SECOND.getTrigger();
        periodic.liftEncoder = lowerLift.get();
        periodic.buttonArray[0] = liftUpper.getSensorCollection().isFwdLimitSwitchClosed();
        periodic.buttonArray[1] = liftUpper.getSensorCollection().isRevLimitSwitchClosed();
        periodic.buttonArray[2] = liftLower.getSensorCollection().isFwdLimitSwitchClosed();
        periodic.buttonArray[3] = liftLower.getSensorCollection().isRevLimitSwitchClosed();
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {

        if (periodic.trigger || LiftDisable()) {
            liftUpper.set(-1.5 * operatorInput[1]);
            liftLower.set(operatorInput[1]);
        } else {
            liftLower.set(operatorInput[1]);
            liftUpper.set(0);
        }
    }

    private boolean LiftDisable() {
        return ((periodic.buttonArray[2] && !periodic.buttonArray[1]) || (periodic.buttonArray[3] && !periodic.buttonArray[0])) &&
                !SmartDashboard.getBoolean("DB/Button 0", false);
    }


    public void outputTelemetry() {
        SmartDashboard.putNumber("LiftEncoder", periodic.liftEncoder);
        SmartDashboard.putNumberArray("LiftUpper", operatorInput);
        SmartDashboard.putBoolean("Stage 1 Low", periodic.buttonArray[3]);
        SmartDashboard.putBoolean("Stage 1 High", periodic.buttonArray[2]);
        SmartDashboard.putBoolean("Stage 2 Low", periodic.buttonArray[0]);
        SmartDashboard.putBoolean("Stage 2 High", periodic.buttonArray[1]);
    }

    public void stop() {
    }

    public void reset() {
        periodic = new PeriodicIO();
        lowerLift.reset();

    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    public static class PeriodicIO {
        //INPUTS
        public boolean trigger = false;
        public int liftEncoder = 0;
        public boolean[] buttonArray = new boolean[4];

        //OUTPUTS

    }
}
