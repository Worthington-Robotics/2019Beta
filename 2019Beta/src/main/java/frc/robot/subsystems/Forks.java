package frc.robot.subsystems;

import frc.lib.loops.ILooper;

public class Forks extends Subsystem{
    public void writeToLog(){}

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {
    }

    public void outputTelemetry(){}

    public void stop(){}

    public void reset(){}

    public void registerEnabledLoops(ILooper enabledLooper){

    }
}
