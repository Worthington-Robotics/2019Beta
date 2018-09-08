package frc.robot.subsystems;

import frc.lib.loops.ILooper;

public abstract class Subsystem {

    public void writeToLog(){}

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {
    }

    public abstract void outputTelemetry();

    public abstract void stop();

    public abstract void reset();

    public void registerEnabledLoops(ILooper enabledLooper){

    }

}
