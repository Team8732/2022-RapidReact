package frc.team8732.lib.trajectory;

import frc.team8732.lib.geometry.State;

public interface TrajectoryView<S extends State<S>> {
    TrajectorySamplePoint<S> sample(final double interpolant);

    double first_interpolant();

    double last_interpolant();

    Trajectory<S> trajectory();
}
