// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.auto.actions;

import frc.team8732.robot.subsystems.Drive;

/** Add your docs here. */
public class MovePercentAction implements Action {

    /** Creates a new setIntakeSystemState. */
    Drive mDrive = Drive.getInstance();
    double percent;
    public MovePercentAction(double percent) {
        this.percent = percent;
    }


    @Override
    public void start() {
        mDrive.setPercentOpenLoop(percent);
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
