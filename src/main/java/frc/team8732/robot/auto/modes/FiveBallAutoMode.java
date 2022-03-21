// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.auto.modes;

import java.util.List;

import frc.team8732.robot.auto.AutoModeEndedException;
import frc.team8732.robot.auto.actions.DriveSystemStateAction;
import frc.team8732.robot.auto.actions.DriveTrajectoryAction;
import frc.team8732.robot.auto.actions.IntakeSystemStateAction;
import frc.team8732.robot.auto.actions.ParallelAction;
import frc.team8732.robot.auto.actions.SeriesAction;
import frc.team8732.robot.auto.actions.SystemIdleAction;
import frc.team8732.robot.auto.actions.SystemSetCalculatedShotAction;
import frc.team8732.robot.auto.actions.WaitAction;
import frc.team8732.robot.paths.TrajectoryGenerator;
import frc.team8732.robot.subsystems.Drive.DriveControlState;
import frc.team8732.robot.subsystems.Intake.IntakeSystemState;

/** Add your docs here. */
public class FiveBallAutoMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new IntakeSystemStateAction(IntakeSystemState.RELEASE)); // Drop Intake
        runAction(new SystemIdleAction()); // Set Hood, Shooter, and Intake Idle
        runAction(new WaitAction(2)); // Wait to spin up and release
        runAction(new IntakeSystemStateAction(IntakeSystemState.INTAKING)); // Start intaking sequence 
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().tarmach1StartToBall2, true)); // Drive first ball pick up
        runAction(new ParallelAction(List.of(
            new SystemSetCalculatedShotAction(), // AIM to goal and set calc RPM + Hood
            new SeriesAction(List.of(
                new WaitAction(1), // Spin Up
                new IntakeSystemStateAction(IntakeSystemState.SHOOTING), // Shoot
                new WaitAction(1.75), // Shoot Timeout
                new DriveSystemStateAction(DriveControlState.PATH_FOLLOWING), // Stop SystemCalcAction
                new SystemIdleAction() // Set Hood, Shooter, and Intake Idle
            ))
        )
        ));


        // // Auto Shot Idle
        runAction(new IntakeSystemStateAction(IntakeSystemState.INTAKING));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().Ball2ToBall1));
        runAction(new WaitAction(1.4)); // Intake
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().Ball1ToShootPose1));
        runAction(new ParallelAction(List.of(
            new SystemSetCalculatedShotAction(), // AIM to goal and set calc RPM + Hood
            new SeriesAction(List.of(
                new WaitAction(1.5), // Spin Up
                new IntakeSystemStateAction(IntakeSystemState.SHOOTING), // Shoot
                new WaitAction(2.5), // Shoot Timeout
                new DriveSystemStateAction(DriveControlState.PATH_FOLLOWING), // Stop SystemCalcAction
                new SystemIdleAction() // Set Hood, Shooter, and Intake Idle
            ))
        )
        )); // Shoot // 2 Balls

        // runAction(new IntakeSystemStateAction(IntakeSystemState.INTAKING));
        // runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().ShootPose1ToBall3));
        // runAction(new SystemCalculatedShotAction(.5));
        // runAction(new IntakeSystemStateAction(IntakeSystemState.SHOOTING));
        // runAction(new WaitAction(1.25)); // Shoot // 1


    }
}
