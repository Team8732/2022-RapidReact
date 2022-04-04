// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team8732.robot.subsystems.Climb;

public class setClimbPercent extends CommandBase {
  /** Creates a new setClimbInches. */
  Climb mClimb = Climb.getInstance();
  double output;
  public setClimbPercent(double output) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.output = output;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {mClimb.setOpenLoop(output);}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
