package frc.team8732.robot.paths;

import frc.team8732.lib.geometry.Pose2d;
import frc.team8732.lib.geometry.Pose2dWithCurvature;
import frc.team8732.lib.geometry.Rotation2d;
import frc.team8732.lib.geometry.Translation2d;
import frc.team8732.lib.trajectory.Trajectory;
import frc.team8732.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.team8732.lib.trajectory.timing.TimedState;
import frc.team8732.lib.trajectory.timing.TimingConstraint;
import frc.team8732.robot.planners.DriveMotionPlanner;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static TrajectoryGenerator mInstance;


    // TODO tune
    private static final double kMaxVel = 160.0; // 150
    private static final double kMaxAccel = 80.0;
    private static final double kMaxVoltage = 9.0;

    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public synchronized static TrajectoryGenerator getInstance() {
        if (mInstance == null) {
            mInstance = new TrajectoryGenerator();
        }

        return mInstance;
    }
    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

        // CRITICAL POSES
        // Origin is the center of the robot when the robot is placed against the middle
        // of the alliance station wall.
        // +x is towards the center of the field.
        // +y is to the left.
        // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x
        // axis for LEFT)

        // 2022 Rapid React Poses

        public static final Translation2d kMidVeihcleToFrontVechile = new Translation2d(23.25, 23.25); //15.5
        public static final Translation2d kHalfBall = new Translation2d(-4.75, -4.75);

        public static final Pose2d kTarmach1StartingPose = new Pose2d(new Translation2d(255, -61), // Conrner of tarmach 1 facing the alliance wall X = 254 Y = -58
                Rotation2d.fromDegrees(-140));

        public static final Pose2d kBall1Pose = new Pose2d(new Translation2d(41.920 + (kMidVeihcleToFrontVechile.x() + kHalfBall.x()), -117.820 + (kMidVeihcleToFrontVechile.y() + kHalfBall.y())), // Ball @ loading zone X = 41.920 Y = -117.820
                Rotation2d.fromDegrees(-133.75));

        public static final Pose2d kBall2Pose = new Pose2d(new Translation2d(199.054 + (kMidVeihcleToFrontVechile.x() + kHalfBall.x()), -88.429205 + (kMidVeihcleToFrontVechile.y() + kHalfBall.y())), // Alliance cargo closest to tarmach 2 X = 199.054 Y = -88.429205
                Rotation2d.fromDegrees(-135));

        public static final Pose2d kBall3Pose = new Pose2d(new Translation2d(298, -133), // Alliance cargo furthest from tarmach 2 X = 298.090 Y = -150.864
                Rotation2d.fromDegrees(-90));

        public static final Pose2d kTarmach1ShootPose = new Pose2d(new Translation2d(292, -70), //296.5 -60
                Rotation2d.fromDegrees(-114));

        // Tarmach 2 (Hanger Zone)
        public static final Pose2d kTarmach2StartingPose = new Pose2d(new Translation2d(237, 45), // Front Bumpers on tape facing Ball 4 & Hanger Zone
                Rotation2d.fromDegrees(135));

        public static final Pose2d kBall4Pose = new Pose2d(new Translation2d(194.603643 + (kMidVeihcleToFrontVechile.x() + kHalfBall.x()), 81.779 - (kMidVeihcleToFrontVechile.y() + kHalfBall.y())), // Alliance cargo closest to tarmach 2 X = 194.603643 Y = 81.779
                Rotation2d.fromDegrees(135));

        public static final Pose2d kBall5PoseOpp = new Pose2d(new Translation2d(235.696786 + (kMidVeihcleToFrontVechile.x() + kHalfBall.x()), 125.036414 - (kMidVeihcleToFrontVechile.y() + kHalfBall.y())), // Alliance cargo closest to tarmach 2 X = 194.603643 Y = 81.779
                Rotation2d.fromDegrees(135));

        public static final Pose2d kBall6PoseOpp = new Pose2d(new Translation2d(174.773 + (kMidVeihcleToFrontVechile.x() + kHalfBall.x()), -34.099 + (kMidVeihcleToFrontVechile.y() + kHalfBall.y())), // Alliance cargo closest to tarmach 2 X = 194.603643 Y = 81.779
                Rotation2d.fromDegrees(-135));

        public static final Pose2d kTarmach2ShootPose = new Pose2d(new Translation2d(230, 33),
                Rotation2d.fromDegrees(157.5));

        public static final Pose2d kBall1T2MidPose = new Pose2d(new Translation2d(160, 15),
                Rotation2d.fromDegrees(-115.5));  

        public static final Pose2d kTarmach2TurningPose = new Pose2d(new Translation2d(258, 26),
                Rotation2d.fromDegrees(157));  

        public static final Pose2d kTarmach2DropOff = new Pose2d(new Translation2d(158, 97),
                Rotation2d.fromDegrees(180));
        
        public static final Pose2d kTarmach2StartToBall3 = new Pose2d(new Translation2d(298, -90),
                Rotation2d.fromDegrees(-90)); 

        public static final Pose2d kShootPoseTerminalMid = new Pose2d(new Translation2d(85, -93),
                Rotation2d.fromDegrees(-154)); 

    public class TrajectorySet {
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTrajectory;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTrajectoryBack;

        public final Trajectory<TimedState<Pose2dWithCurvature>> tarmach1StartToBall2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Ball2ToBall1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Ball1ToShootPose1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> ShootPoseToBall3;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Ball3ToShootPose1;

        public final Trajectory<TimedState<Pose2dWithCurvature>> tarmach2StartToBall4;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Ball4ToShootPose2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> ShootPose2ToBall1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Ball1ToShootPose2;
       
        public final Trajectory<TimedState<Pose2dWithCurvature>> Ball4ToT2TurningPose;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Tarmach2TurningPoseTokBall6PoseOpp;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Tarmach2TurningPoseToBall1;

        
        public final Trajectory<TimedState<Pose2dWithCurvature>> Ball6PoseOppToTarmach2TurningPose;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Tarmach2TurningPoseTokBall5PoseOpp;
        public final Trajectory<TimedState<Pose2dWithCurvature>> Ball5PoseOppToTarmach2DropOff;

        // public final Trajectory<TimedState<Pose2dWithCurvature>> Tarmach2StartToBall3;


        private TrajectorySet() {
            testTrajectory = getTestTrajectory();
            testTrajectoryBack = getTestTrajectoryBack();

            tarmach1StartToBall2 = getTarmach1StartToBall2();
            Ball2ToBall1 = getBall2ToBall1();
            Ball1ToShootPose1 = getBall1ToShootPose1();
            ShootPoseToBall3 = getShootPoseToBall3();
            Ball3ToShootPose1 = getBall3ToShootPose1();

            tarmach2StartToBall4 = getTarmach2ToBall4();
            Ball4ToShootPose2 = getBall4ToShootPose2();
            ShootPose2ToBall1 = getShootPose2ToBall1();
            Ball1ToShootPose2 = getBall1ToShootPose2();

            Ball4ToT2TurningPose = getBall4ToT2TurningPose();
            Tarmach2TurningPoseTokBall6PoseOpp = getTarmach2TurningPoseTokBall6PoseOpp();

            Ball6PoseOppToTarmach2TurningPose = getBall6PoseOppToTarmach2TurningPose();
            Tarmach2TurningPoseToBall1 =  getTurnPoseToBall1();
            Tarmach2TurningPoseTokBall5PoseOpp = getTarmach2TurningPoseTokBall5PoseOpp();
            Ball5PoseOppToTarmach2DropOff = getBall5PoseOppToTarmach2DropOff();

        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-120, 120, Rotation2d.fromDegrees(90)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectoryBack() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-120, 120, Rotation2d.fromDegrees(90)));
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        // 5 Ball Auto Trajectory 

        private Trajectory<TimedState<Pose2dWithCurvature>> getTarmach1StartToBall2() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kTarmach1StartingPose);
                waypoints.add(kBall2Pose);

                return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBall2ToBall1() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kBall2Pose);
                waypoints.add(kBall1Pose);

                return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBall1ToShootPose1() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kBall1Pose);
                waypoints.add(kTarmach1ShootPose);

                return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getShootPoseToBall3() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kTarmach1ShootPose);
                waypoints.add(kBall3Pose);

                return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBall3ToShootPose1() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kBall3Pose);
                waypoints.add(kTarmach1ShootPose);

                return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }

    
    private Trajectory<TimedState<Pose2dWithCurvature>> getTarmach2ToBall4() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTarmach2StartingPose);
            waypoints.add(kBall4Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
    }
    
    private Trajectory<TimedState<Pose2dWithCurvature>> getBall4ToShootPose2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kBall4Pose);
            waypoints.add(kTarmach2ShootPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
            kMaxVel, kMaxAccel, kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getShootPose2ToBall1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTarmach2ShootPose);
            waypoints.add(kBall1Pose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
            kMaxVel, kMaxAccel, kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getBall1ToShootPose2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kBall1Pose);
            waypoints.add(kShootPoseTerminalMid);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
            kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBall4ToT2TurningPose() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kBall4Pose);
                waypoints.add(kTarmach2TurningPose);

                return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTarmach2TurningPoseTokBall6PoseOpp() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kTarmach2TurningPose);
                waypoints.add(kBall6PoseOpp);

                return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getBall6PoseOppToTarmach2TurningPose() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kBall6PoseOpp);
                waypoints.add(kTarmach2TurningPose);

                return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTurnPoseToBall1() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kTarmach2TurningPose);
                waypoints.add(kBall1Pose);

                return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTarmach2TurningPoseTokBall5PoseOpp() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kTarmach2TurningPose);
                waypoints.add(kBall5PoseOpp);

                return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBall5PoseOppToTarmach2DropOff() {
                List<Pose2d> waypoints = new ArrayList<>();
                waypoints.add(kBall5PoseOpp);
                waypoints.add(kTarmach2DropOff);

                return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                kMaxVel, kMaxAccel, kMaxVoltage);
        }
        }
}