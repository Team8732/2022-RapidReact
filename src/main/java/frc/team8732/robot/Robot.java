// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team8732.lib.geometry.Pose2d;
import frc.team8732.lib.geometry.Rotation2d;
import frc.team8732.lib.util.CrashTracker;
import frc.team8732.robot.auto.AutoModeExecutor;
import frc.team8732.robot.auto.modes.AutoModeBase;
import frc.team8732.robot.loops.Looper;
import frc.team8732.robot.paths.TrajectoryGenerator;
import frc.team8732.robot.subsystems.Drive;
import frc.team8732.robot.subsystems.Drive.DriveControlState;
import frc.team8732.robot.subsystems.Hood;
import frc.team8732.robot.subsystems.Intake;
import frc.team8732.robot.subsystems.RobotStateEstimator;
import frc.team8732.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();
  
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  private final AutoModeSelector mAutoModeSelector = new AutoModeSelector();
  private AutoModeExecutor mAutoModeExecutor;

  private final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

  private double mDisabledStartTime = Double.NaN;

  // Subsystems
  private final Drive mDrive = Drive.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Intake mIntake = Intake.getInstance();
  private final Hood mHood = Hood.getInstance();
  private final Limelight mLimelight = Limelight.getInstance();

  // Robot State
  private final RobotState mRobotState = RobotState.getInstance();

  // Constructor
  Robot() {
    CrashTracker.logRobotConstruction();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // RobotContainer.getInstance();

    try {
        CrashTracker.logRobotInit();

        mSubsystemManager.setSubsystems(
          RobotStateEstimator.getInstance(),
          mDrive,
          mShooter,
          mIntake,
          mHood,
          mLimelight
        );

        mSubsystemManager.registerEnabledLoops(mEnabledLooper);
        mSubsystemManager.registerDisabledLoops(mDisabledLooper);

        mTrajectoryGenerator.generateTrajectories();

        // Robot starts backwards, turret starts backwards (in robot frame)
        mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.fromRotation(Rotation2d.fromDegrees(180)));
        mDrive.setHeading(Rotation2d.fromDegrees(180)); // TODO Check Heading

        mAutoModeSelector.updateModeCreator();

        mSubsystemManager.stop();

      } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    try {
      CrashTracker.logDisabledInit();
      mEnabledLooper.stop();

      // Reset all auto mode state.
      if (mAutoModeExecutor != null) {
          mAutoModeExecutor.stop();
      }
      mAutoModeSelector.reset();
      mAutoModeSelector.updateModeCreator();
      mAutoModeExecutor = new AutoModeExecutor();

      mDisabledLooper.start();

      mDisabledStartTime = Timer.getFPGATimestamp();

  } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    try {
      CrashTracker.logAutoInit();

      mDisabledLooper.stop();
    
      // Robot starts backwards, turret starts backwards (in robot frame)
      mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.fromRotation(Rotation2d.fromDegrees(180)));
      mDrive.setHeading(Rotation2d.fromDegrees(180)); //TODO Check Heading

      mEnabledLooper.start();
      mAutoModeExecutor.start();

  } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void teleopInit() {
    try {
    
      CrashTracker.logTeleopInit();
      mDisabledLooper.stop();

      if (mAutoModeExecutor != null) {
          mAutoModeExecutor.stop();
      }

      mSubsystemManager.stop();

      mDrive.setControlState(DriveControlState.JOYSTICK);

      mEnabledLooper.start();

  } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    try {
      CrashTracker.logTestInit();
      System.out.println("Starting check systems.");

      mDisabledLooper.stop();
      mEnabledLooper.stop();

      if (mDrive.checkSystem()) {
          System.out.println("ALL SYSTEMS PASSED");
      } else {
          System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
      }
  } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, tele-operated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    try {
        mSubsystemManager.outputToSmartDashboard();
        RobotState.getInstance().outputToSmartDashboard();
        mAutoModeSelector.outputToSmartDashboard();
    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    try {
      // Update auto modes
      mAutoModeSelector.updateModeCreator();

      Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
      if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
          System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
          mAutoModeExecutor.setAutoMode(autoMode.get());
      }

      if ((Timer.getFPGATimestamp() - mDisabledStartTime) > 5.0 &&
              (Timer.getFPGATimestamp() - mDisabledStartTime) < 5.5) {
          System.out.println("Setting coast!");
          mDrive.setBrakeMode(false);
        }
      } catch (Throwable t) {
          CrashTracker.logThrowableCrash(t);
          throw t;
      }
    } 

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}
