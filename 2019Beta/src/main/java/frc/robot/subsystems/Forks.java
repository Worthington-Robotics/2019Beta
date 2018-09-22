package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

public class Forks extends Subsystem {
    //The one instance of Lift
    private static Forks m_ForksInstance = new Forks();
    private double[] operatorInput = {0, 0, 0};
    private PeriodicIO periodic;
    private Spark rightShooter;
    private Spark leftShooter;
    private Spark forkUD;
    private final Loop mLoop = new Loop() {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {

        }

        public void onStop(double timestamp) {
            forkUD.set(0);
            leftShooter.set(0);
            rightShooter.set(0);
        }
    };

    public Forks() {
        periodic = new PeriodicIO();
        leftShooter = new Spark(Constants.LEFT_SHOOTER_ID);
        rightShooter = new Spark(Constants.RIGHT_SHOOTER_ID);
        forkUD = new Spark(Constants.FORKSUD_ID);
    }

    public static Forks getInstance() {
        return m_ForksInstance;
    }

    public void setShotPower(double Power) {
        periodic.ShotPower = Power;
    }

    public void setUDPower(double Power) {
        periodic.UDpower = Power;
    }

    public void writeToLog() {
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {

    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {
        leftShooter.set(periodic.ShotPower);
        rightShooter.set(-periodic.ShotPower);
        forkUD.set(periodic.UDpower);

    }


    public void outputTelemetry() {
    }

    public void stop() {
    }

    public void reset() {
        periodic = new PeriodicIO();

    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    public static class PeriodicIO {
        //INPUTS
        public int liftEncoder = 0;
        //OUTPUTS
        public double ShotPower = 0.0;
        public double UDpower = 0.0;
    }
}
