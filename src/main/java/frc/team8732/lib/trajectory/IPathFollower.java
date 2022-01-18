package frc.team8732.lib.trajectory;

import frc.team8732.lib.geometry.Pose2d;
import frc.team8732.lib.geometry.Twist2d;

public interface IPathFollower {
    Twist2d steer(Pose2d current_pose);

    boolean isDone();
}
