package frc.team8732.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team8732.lib.geometry.Pose2dWithCurvature;
import frc.team8732.lib.trajectory.TimedView;
import frc.team8732.lib.trajectory.Trajectory;
import frc.team8732.lib.trajectory.TrajectoryIterator;
import frc.team8732.lib.trajectory.timing.TimedState;
import frc.team8732.robot.RobotState;
import frc.team8732.robot.subsystems.Drive;
import frc.team8732.robot.subsystems.Drive.DriveControlState;

public class DriveTrajectoryCommand extends CommandBase {
    private static final Drive mDrive = Drive.getInstance();
    private static final RobotState mRobotState = RobotState.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final boolean mResetPose;

    public DriveTrajectoryCommand(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        this(trajectory, false);
    }


    public DriveTrajectoryCommand(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
    }

    @Override
    public void initialize() {
        mDrive.setControlState(DriveControlState.PATH_FOLLOWING);
        System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose) {
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
        }
        mDrive.setTrajectory(mTrajectory);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            System.out.println("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}

