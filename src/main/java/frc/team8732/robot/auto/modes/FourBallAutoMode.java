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
public class FourBallAutoMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new IntakeSystemStateAction(IntakeSystemState.RELEASE)); // Drop Intake
        runAction(new WaitAction(.5)); // Wait to spin up and release
        runAction(new SystemIdleAction()); // Set Hood, Shooter, and Intake Idle
        runAction(new IntakeSystemStateAction(IntakeSystemState.INTAKING)); // Start intaking sequence 
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().tarmach2StartToBall4, true)); // Drive first ball pick up
        runAction(new ParallelAction(List.of(
            new SystemSetCalculatedShotAction(), // AIM to goal and set calc RPM + Hood
            new SeriesAction(List.of(
                new WaitAction(.75), // Spin Up
                new IntakeSystemStateAction(IntakeSystemState.SHOOTING), // Shoot
                new WaitAction(1.75), // Shoot Timeout
                new DriveSystemStateAction(DriveControlState.PATH_FOLLOWING), // Stop SystemCalcAction
                new SystemIdleAction() // Set Hood, Shooter, and Intake Idle
            ))
        )
        ));

        // Auto Shot Idle
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().Ball4ToT2TurningPose));
        runAction(new IntakeSystemStateAction(IntakeSystemState.INTAKING));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().Tarmach2TurningPoseToBall1));
        runAction(new WaitAction(.25)); // Intake
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().Ball1ToShootPose2));
        runAction(new ParallelAction(List.of(
            new SystemSetCalculatedShotAction(), // AIM to goal and set calc RPM + Hood
            new SeriesAction(List.of(
                new WaitAction(.35), // Spin Up
                new IntakeSystemStateAction(IntakeSystemState.SHOOTING_AUTO), // Shoot
                new WaitAction(2.5), // Shoot Timeout
                new DriveSystemStateAction(DriveControlState.PATH_FOLLOWING), // Stop SystemCalcAction
                new SystemIdleAction() // Set Hood, Shooter, and Intake Idle
            ))
        )
        ));
    }
}
