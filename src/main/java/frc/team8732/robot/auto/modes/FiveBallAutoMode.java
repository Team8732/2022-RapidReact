// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.auto.modes;

import frc.team8732.robot.auto.AutoModeEndedException;
import frc.team8732.robot.auto.actions.DriveTrajectoryAction;
import frc.team8732.robot.auto.actions.WaitAction;
import frc.team8732.robot.auto.actions.setIntakeSystemStateAction;
import frc.team8732.robot.paths.TrajectoryGenerator;
import frc.team8732.robot.subsystems.Intake.IntakeSystemState;

/** Add your docs here. */
public class FiveBallAutoMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new setIntakeSystemStateAction(IntakeSystemState.RELEASE));
        runAction(new WaitAction(.75)); //  Release
        runAction(new setIntakeSystemStateAction(IntakeSystemState.INTAKING));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().tarmach1StartToBall2));
        // new SystemSetDefinedShot(Constants.kShooterProtectedRPM, Constants.kHoodProtectedDegree); Make Auto Shot
        runAction(new setIntakeSystemStateAction(IntakeSystemState.SHOOTING));
        runAction(new WaitAction(1.25)); // Shoot
        // Auto Shot Idle
        runAction(new setIntakeSystemStateAction(IntakeSystemState.INTAKING));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().Ball2ToBall1));
        runAction(new WaitAction(.75)); // Intake
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().Ball1ToShootPose1));
        // new SystemSetDefinedShot(Constants.kShooterProtectedRPM, Constants.kHoodProtectedDegree); Make Auto Shot
        runAction(new setIntakeSystemStateAction(IntakeSystemState.SHOOTING));
        runAction(new WaitAction(1.25)); // Shoot
        runAction(new setIntakeSystemStateAction(IntakeSystemState.INTAKING));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().ShootPose1ToBall3));

    }
}
