package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class Forks extends Subsystem {
    //The one instance of Lift
    private static Forks m_ForksInstance = new Forks();
    private PeriodicIO periodic;
    private Spark rightShooter;
    private Spark leftShooter;
    private Spark forkUD;
    private final Loop mLoop = new Loop() {
        public void onStart(double timestamp) {

        }

        public void onLoop(double timestamp) {
            if (periodic.B2) {
                setShotPower(-1);
            } else if (periodic.B3) {
                setShotPower(1);
            } else if (periodic.B4) {
                setShotPower(0.5);
            } else if (periodic.B5) {
                setShotPower(-0.5);
            } else {
                setShotPower(0.0);
            }
            if (periodic.B8) {
                setUDPower(1.0);
            } else if(periodic.B9) {
                setUDPower(-1.0);
            } else {
                setUDPower(0.0);

            }
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

    public void writeToLog() {
    }

    public void setShotPower(double power){
        periodic.shotPower = power;
    }

    public void setUDPower(double power){
        periodic.udPower = power;
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {
        periodic.B2 = Constants.SECOND.getRawButton(2);
        periodic.B3 = Constants.SECOND.getRawButton(3);
        periodic.B4 = Constants.SECOND.getRawButton(4);
        periodic.B5 = Constants.SECOND.getRawButton(5);
        periodic.B8 = Constants.SECOND.getRawButton(8);
        periodic.B9 = Constants.SECOND.getRawButton(9);

    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {
        leftShooter.set(periodic.shotPower);
        rightShooter.set(-periodic.shotPower);
        forkUD.set(periodic.udPower);
    }


    public void outputTelemetry() {
        SmartDashboard.putBoolean("Button 9", periodic.B9);
        SmartDashboard.putBoolean("Button 8", periodic.B8);
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
        public boolean B2 = false;
        public boolean B3 = false;
        public boolean B4 = false;
        public boolean B5 = false;
        public boolean B8 = false;
        public boolean B9 = false;
        public int liftEncoder = 0;
        public double shotPower = 0.0;
        public double udPower = 0.0;
        //OUTPUTS

    }
}
