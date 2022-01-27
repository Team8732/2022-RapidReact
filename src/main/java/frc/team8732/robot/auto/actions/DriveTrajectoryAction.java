package frc.team8732.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.team8732.lib.geometry.Pose2dWithCurvature;
import frc.team8732.lib.trajectory.TimedView;
import frc.team8732.lib.trajectory.Trajectory;
import frc.team8732.lib.trajectory.TrajectoryIterator;
import frc.team8732.lib.trajectory.timing.TimedState;
import frc.team8732.robot.RobotState;
import frc.team8732.robot.subsystems.Drive;

public class DriveTrajectoryAction implements Action {
    private static final Drive mDrive = Drive.getInstance();
    private static final RobotState mRobotState = RobotState.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final boolean mResetPose;

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        this(trajectory, false);
    }


    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
    }

    @Override
    public void start() {
        System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose) {
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
        }
        mDrive.setTrajectory(mTrajectory);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            System.out.println("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void done() {}
}

