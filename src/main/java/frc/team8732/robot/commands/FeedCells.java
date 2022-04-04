// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team8732.robot.subsystems.Hood;
import frc.team8732.robot.subsystems.Intake;
import frc.team8732.robot.subsystems.Shooter;
import frc.team8732.robot.subsystems.Intake.IntakeSystemState;

public class FeedCells extends CommandBase {
  /** Creates a new FeedCells. */
  Intake mIntake = Intake.getInstance();
  Shooter mShooter = Shooter.getInstance();
  Hood mHood = Hood.getInstance();
  public FeedCells() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mShooter.isAtSetpoint() && mHood.isAtSetpoint()){
      mIntake.setSystemState(IntakeSystemState.SHOOTING);
    }else if(mShooter.isAtSetpoint()){
      mIntake.setSystemState(IntakeSystemState.SHOOTING);
    }else{
      mIntake.setSystemState(IntakeSystemState.IDLE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
