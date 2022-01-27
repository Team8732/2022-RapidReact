package frc.team8732.robot.auto.modes;

import frc.team8732.robot.auto.AutoModeEndedException;
import frc.team8732.robot.auto.actions.DriveTrajectoryAction;
import frc.team8732.robot.paths.TrajectoryGenerator;

public class TestTrajectoryFollowingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectoryBack));
    }
}