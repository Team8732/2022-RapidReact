// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team8732.robot.Constants;
import frc.team8732.robot.subsystems.Drive;
import frc.team8732.robot.subsystems.Hood;
import frc.team8732.robot.subsystems.Shooter;
import frc.team8732.robot.subsystems.Drive.DriveControlState;

public class SystemSetDefinedShot extends CommandBase {
  /** Creates a new SystemSetDefinedShot. */
  Shooter mShooter = Shooter.getInstance();
  Hood mHood = Hood.getInstance();
  Drive mDrive =  Drive.getInstance();

  double rpm;
  double degree;
  public SystemSetDefinedShot(double rpm, double degree) {
    this.rpm = rpm;
    this.degree = degree;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrive.setControlState(DriveControlState.LIMELIGHT);
    mShooter.setRPM(rpm);
    mHood.setDegree(degree);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // mShooter.setWantsShoot(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setRPM(Constants.kShooterIdleRPM);
    mHood.setDegree(Constants.kHoodIdleDegree);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mDrive.getControlState() == DriveControlState.JOYSTICK;
  }
}