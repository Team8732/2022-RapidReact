// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.auto.actions;

import frc.team8732.robot.subsystems.Intake;
import frc.team8732.robot.subsystems.Intake.IntakeSystemState;

/** Add your docs here. */
public class IntakeSystemStateAction implements Action {

    /** Creates a new setIntakeSystemState. */
    Intake mIntake = Intake.getInstance();
    IntakeSystemState systemState;
    public IntakeSystemStateAction(IntakeSystemState systemState) {
        this.systemState = systemState;
    }


    @Override
    public void start() {
        mIntake.setSystemState(systemState);
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
