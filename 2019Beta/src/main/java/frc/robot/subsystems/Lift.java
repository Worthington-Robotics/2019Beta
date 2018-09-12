package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class Lift extends Subsystem {
    //The one instance of Lift
    private static Lift m_LiftInstance = new Lift();
    private double[] operatorInput = {0, 0, 0};
    private WPI_TalonSRX liftLower;
    private WPI_TalonSRX liftUpper;
    private final Loop mLoop = new Loop() {
        public void onStart(double timestamp) {
        }

        public void onLoop(double timestamp) {
            setOperatorInput(HIDHelper.getAdjStick(SECOND_STICK));
            if(SECOND.getTrigger()) liftUpper.set(operatorInput[1]);

            liftLower.set(1);
        }

        public void onStop(double timestamp) {
        liftLower.set(0);
        liftUpper.set(0);
        }
    };
    public Lift()
    {
    liftLower = new WPI_TalonSRX(LIFT_LOWER_ID);
    liftUpper = new WPI_TalonSRX(LIFT_UPPER_ID);
    }
    public void writeToLog() {
    }
    public void setOperatorInput(double[] input) {
        operatorInput = HIDHelper.getAdjStick(SECOND_STICK);
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {
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
}
