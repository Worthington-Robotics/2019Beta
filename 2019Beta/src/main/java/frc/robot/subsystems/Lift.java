package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;

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
        }

        public void onStop(double timestamp) {

        }
    };
    public Lift()
    {

    }
    public void writeToLog() {
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
