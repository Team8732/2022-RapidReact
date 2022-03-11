// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot;

import frc.team8732.robot.controller.GameController;
import frc.team8732.robot.controller.Playstation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer mInstance;

  private final GameController mDriver = new GameController(Constants.kDriveGamepadPort, new Playstation());
  private final GameController mOperator = new GameController(Constants.kButtonGamepadPort, new Playstation());

  public synchronized static RobotContainer getInstance() {
    if (mInstance == null) {
        mInstance = new RobotContainer();
    }

    return mInstance;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button-> command mappings.
   */
  private void configureButtonBindings() {}

  public GameController getDriveGameController(){
    return mDriver;
  }

  public GameController getOperatorGameController(){
    return mOperator;
  }
}
