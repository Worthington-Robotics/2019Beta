package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.SubsystemManager;

import static frc.robot.Constants.*;

public class Lift extends Subsystem {
    //The one instance of Lift
    private static Lift m_LiftInstance = new Lift();
    private double[] operatorInput = {0, 0, 0};
    private PeriodicIO periodic;
    private WPI_TalonSRX liftLower;
    private WPI_TalonSRX liftUpper;
    public static Lift mLIftInstance = new Lift();
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
        reset();
        liftLower = new WPI_TalonSRX(LIFT_LOWER_ID);
        liftUpper = new WPI_TalonSRX(LIFT_UPPER_ID);
    }
    public static Lift getInstance(){return mLIftInstance;}

    public void writeToLog() {
    }

    public void setOperatorInput(double[] input) {
        operatorInput = input;
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {
        setOperatorInput(HIDHelper.getAdjStick(SECOND_STICK));
        PeriodicIO.trigger = SECOND.getTrigger();
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {

        if (PeriodicIO.trigger) {
            liftUpper.set(operatorInput[1]);
        } else {
            liftLower.set(operatorInput[1]);
        }
    }


    public void outputTelemetry() {

    }

    public void stop() {
    }

    public void reset() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    public static class PeriodicIO {
        //INPUTS
        public static boolean trigger = false;
        //OUTPUTS

    }
}
