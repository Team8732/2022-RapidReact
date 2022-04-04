// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.auto.actions;

import frc.team8732.robot.Constants;
import frc.team8732.robot.subsystems.Hood;
import frc.team8732.robot.subsystems.Intake;
import frc.team8732.robot.subsystems.Shooter;
import frc.team8732.robot.subsystems.Intake.IntakeSystemState;

/** Add your docs here. */
public class SystemIdleAction implements Action {
    
    /** Creates a new setIntakeSystemState. */
    Intake mIntake = Intake.getInstance();
    Shooter mShooter = Shooter.getInstance();
    Hood mHood = Hood.getInstance();
    IntakeSystemState systemState;
    public SystemIdleAction() {
    }


    @Override
    public void start() {
        mIntake.setSystemState(IntakeSystemState.IDLE);
        mShooter.setRPM(Constants.kShooterIdleRPMAuto);
        mHood.setDegree(Constants.kHoodIdleDegree);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}
