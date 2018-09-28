/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.loops.Looper;
import frc.lib.statemachine.StateMachine;
import frc.lib.util.DriveSignal;
import frc.robot.autoactiongroups.CrossTheLine;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Forks;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.RobotStateEstimator;

import java.util.Arrays;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static OI m_oi;
    private final SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(
            RobotStateEstimator.getInstance(),
            Drive.getInstance(),
            Lift.getInstance(),
            Forks.getInstance()
    ));
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        mSubsystemManager.registerEnabledLoops(mEnabledLooper);
        mSubsystemManager.registerDisabledLoops(mDisabledLooper);
        mDisabledLooper.start();
        m_oi = new OI();
        SmartDashboard.putData("Auto mode", m_chooser);
        Drive.getInstance().reset();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        mSubsystemManager.outputTelemetry();
        RobotState.getInstance().outputTelemetry();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
        mEnabledLooper.stop();
        mDisabledLooper.start();

    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional actions to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & actions.
     */
    @Override
    public void autonomousInit() {
        mDisabledLooper.stop();
        mEnabledLooper.start();
        Drive.getInstance().reset();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
        Drive.getInstance().startLogging();
        StateMachine.runMan(new CrossTheLine());

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        mDisabledLooper.stop();
        mEnabledLooper.start();
        Drive.getInstance().reset();
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    public void testInit()
    {
        mDisabledLooper.stop();
        mEnabledLooper.start();
        Drive.getInstance().reset();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }
    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
