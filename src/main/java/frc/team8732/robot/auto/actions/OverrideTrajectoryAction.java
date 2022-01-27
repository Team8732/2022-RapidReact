package frc.team8732.robot.auto.actions;

import frc.team8732.robot.subsystems.Drive;

public class OverrideTrajectoryAction extends RunOnceAction {
    @Override
    public void runOnce() {
        Drive.getInstance().overrideTrajectory(true);
    }
}