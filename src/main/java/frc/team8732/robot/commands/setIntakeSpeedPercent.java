// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.team8732.robot.commands;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team8732.robot.subsystems.Intake;
 
public class setIntakeSpeedPercent extends CommandBase {
  private final Intake mIntake = Intake.getInstance();
 
 
 
 
 
  /** Creates a new setIntakeSpeedPercent. */
  double percentOutput;
  public setIntakeSpeedPercent(double percentOutput){
 // Use addRequirements() here to declare subsystem dependencies.
 this.percentOutput = percentOutput;
  }
 
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {mIntake.setIntakeSpeedPercent(percentOutput);}
 
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
 

