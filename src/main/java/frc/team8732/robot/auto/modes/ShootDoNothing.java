// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.auto.modes;
import java.util.List;

import frc.team8732.robot.auto.AutoModeEndedException;
import frc.team8732.robot.auto.actions.DriveSystemStateAction;
import frc.team8732.robot.auto.actions.IntakeSystemStateAction;
import frc.team8732.robot.auto.actions.MovePercentAction;
import frc.team8732.robot.auto.actions.ParallelAction;
import frc.team8732.robot.auto.actions.SeriesAction;
import frc.team8732.robot.auto.actions.SystemIdleAction;
import frc.team8732.robot.auto.actions.SystemSetCalculatedShotAction;
import frc.team8732.robot.auto.actions.WaitAction;
import frc.team8732.robot.subsystems.Drive.DriveControlState;
import frc.team8732.robot.subsystems.Intake.IntakeSystemState;
import frc.team8732.robot.subsystems.Shooter;


/** Add your docs here. */
public class ShootDoNothing extends AutoModeBase {
    Shooter mShooter = Shooter.getInstance();
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new IntakeSystemStateAction(IntakeSystemState.RELEASE)); // Drop Intake
        runAction(new WaitAction(.5)); // Wait to spin up and release
        runAction(new SystemIdleAction()); // Set Hood, Shooter, and Intake Idle
        runAction(new IntakeSystemStateAction(IntakeSystemState.INTAKING)); // Drop Intake
        runAction(new MovePercentAction(.4));
        runAction(new WaitAction(1.25));
        runAction(new MovePercentAction(0));
        runAction(new ParallelAction(List.of(
            new SystemSetCalculatedShotAction(), // AIM to goal and set calc RPM + Hood
            new SeriesAction(List.of(
                new WaitAction(1.5), // Spin Up
                new IntakeSystemStateAction(IntakeSystemState.SHOOTING), // Shoot
                new WaitAction(2), // Shoot Timeout
                new DriveSystemStateAction(DriveControlState.PATH_FOLLOWING), // Stop SystemCalcAction
                new SystemIdleAction() // Set Hood, Shooter, and Intake Idle
            ))
        )
        ));





        
    }
}
