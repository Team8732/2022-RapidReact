// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.auto.actions;

import frc.team8732.robot.subsystems.Drive;
import frc.team8732.robot.subsystems.Drive.DriveControlState;

/** Add your docs here. */
public class DriveSystemStateAction implements Action {

    /** Creates a new setIntakeSystemState. */
    Drive mDrive = Drive.getInstance();
    DriveControlState systemState;
    public DriveSystemStateAction(DriveControlState systemState) {
        this.systemState = systemState;
    }


    @Override
    public void start() {
        mDrive.setControlState(systemState);
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
