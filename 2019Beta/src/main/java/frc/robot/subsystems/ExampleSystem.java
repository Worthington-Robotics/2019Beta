package frc.robot.subsystems;

import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;

public class ExampleSystem extends Subsystem {

    private static final ExampleSystem m_instance = new ExampleSystem();
    private final Loop mLoop = new Loop() {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {

        }

        public void onStop(double timestamp) {

        }
    };

    public static ExampleSystem getInstance(){
        return m_instance;
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

    }

    public static class PeriodicIO{
        //input variables
        //output variables
    }
}
