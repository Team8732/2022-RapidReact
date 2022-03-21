// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.auto.actions;

import frc.team8732.robot.Constants;
import frc.team8732.robot.Limelight;
import frc.team8732.robot.subsystems.Drive;
import frc.team8732.robot.subsystems.Drive.DriveControlState;
import frc.team8732.robot.subsystems.Hood;
import frc.team8732.robot.subsystems.Shooter;

public class SystemSetCalculatedShotAction implements Action {
  /** Creates a new SystemSetCalculatedShot. */
  Shooter mShooter = Shooter.getInstance();
  Hood mHood = Hood.getInstance();
  Drive mDrive = Drive.getInstance();
  Limelight mLimelight = Limelight.getInstance();

  double rpm;
  double lastValidRPM = -1;

  double degree;
  double lastValidDegree = -1;

  public SystemSetCalculatedShotAction() {}

  // Called when the command is initially scheduled.
  @Override
  public void start() {
    mDrive.setControlState(DriveControlState.AUTO);
    mShooter.setRPM(Constants.kShooterIdleRPM);
    mHood.setDegree(Constants.kHoodIdleDegree);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void update() {
    if(mLimelight.hasTarget()){
      rpm = mShooter.getCalculatedRPM();
      mShooter.setRPM(rpm);
      lastValidRPM = rpm;

      degree = mHood.getCalculatedDegree();
      mHood.setDegree(degree);
      lastValidDegree = degree;
    }else{
      rpm = lastValidRPM != -1 ? lastValidRPM : Constants.kShooterIdleRPM;
      degree = lastValidDegree != -1 ? lastValidDegree : Constants.kHoodIdleDegree;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void done() {
    mShooter.setRPM(Constants.kShooterIdleRPM);
    mHood.setDegree(Constants.kHoodIdleDegree);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mDrive.getControlState() == DriveControlState.PATH_FOLLOWING;
  }
}
