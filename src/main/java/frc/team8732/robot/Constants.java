package frc.team8732.robot;

import edu.wpi.first.math.util.Units;
import frc.team8732.lib.geometry.Translation2d;
import frc.team8732.lib.util.InterpolatingDouble;
import frc.team8732.lib.util.InterpolatingTreeMap;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 * <p>
 * Port assignments should match up with the spreadsheet here:
 * 
 */
public class Constants {
    // ROBOT MAP
    // Drive ID's
    public static final int kLeftDriveMasterID = 0;
    public static final int kLeftDriveSlaveAID = 1;
    public static final int kLeftDriveSlaveBID = 2;
    public static final int kRightDriveMasterID = 15;
    public static final int kRightDriveSlaveAID = 14;
    public static final int kRightDriveSlaveBID = 13;

    // Intake
    public static final int kIntakeGroundID = 5;
    public static final int kIntakeExtensionID = 10; 
    public static final int kTowerTopBeamBreakID = 0; 
    public static final int kTowerBottomBeamBreakID = 2; 

    // Shooter ID's 
    public static final int kShooterMasterID = 3;
    public static final int kShooterSlaveID = 12;
    public static final int kShooterIndexerID = 4;

    // Hood
    public static final int kHoodMotorID = 11;

    // Climb 
    public static final int kClimbMasterID = 7;
    public static final int kClimbSlaveID = 8;

    public static final double kLooperDt = 0.01;

    // CAN
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    // Controllers
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 1;

    // Controller Deadband
    public static final double kDriveThrottleDeadband = 0.02;
    public static final double kDriveWheelDeadband = 0.035;

    // Drive ratio.
    public static final double kDriveEncoderPPR = 4096.0;
    public static final double kDriveGearReduction = 50.0 / 13.0 * 44.0 / 30.0;
    public static final double kGearRatioScalar = (1.0 / (50.0 / 13.0 * 44.0 / 30.0)) / (1.0 / kDriveGearReduction); // TODO Check Scalar

    // Wheel parameters.
    public static final double kDriveWheelTrackWidthInches = 22.5625;              
    public static final double kDriveWheelDiameterInches = 4; //3.90670540497; 
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(kDriveWheelDiameterInches);
    public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kTrackScrubFactor = 0.97;

    // PIDF gains (TODO tune)
    public static final double kDriveKp = 0.15;
    public static final double kDriveKi = 0.0;
    public static final double kDriveKd = 0.0;
    public static final double kDriveKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    public static final double kDrivePositionKp = 0.006;
    public static final double kDrivePositionKi = 0.0;
    public static final double kDrivePositionKd = 0.0;
    public static final double kDrivePositionKf = 0.0;

    // robot dynamics TODO tune
    public static final double kDriveVIntercept = 0.352; // V TODO
    public static final double kDriveLinearKv = 0.11862064541505624 / 2.0 * Constants.kDriveWheelDiameterInches; // V / rad/s
    public static final double kMiniCIMStallTorque = 1.41; // N*m
    public static final double kAssumedTorqueEfficiency = 0.70;
    public static final double kRobotLinearInertia = 48; // kg TODO
    public static final double kDriveAnalyticalLinearKa = 12.0 /* V */ / ((kDriveGearReduction * kMiniCIMStallTorque * kAssumedTorqueEfficiency * 6) / (kRobotLinearInertia * kDriveWheelRadiusMeters * kDriveWheelRadiusMeters));
    public static final double kDriveLinearKa = 0.015470549986407171 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V / rad/s^2
    public static final double kDriveAngularKa = 0.00517 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V per rad/s^2
    public static final double kRobotAngularInertia = kDriveAngularKa / kDriveLinearKa *
            kDriveWheelTrackRadiusWidthMeters * kDriveWheelTrackRadiusWidthMeters * kRobotLinearInertia;  // kg m^2
    public static final double kRobotAngularDrag = 15.0; // N*m / (rad/sec)

    // path following TODO tune?
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    public static final Translation2d kVehicleToTurretTranslation = new Translation2d(-6.9, 0);

    // Shooter
    public static final double kShooterKp = 0.0067;
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 0.0;
    public static final double kShooterKf = 0.03873127788;
    public static final double kShooterEncoderPPR = 4096.0;
    public static final double kShooterOutputToEncoderRatio = 1.0;
    public static final double kShooterTicksPerRevolution = kShooterOutputToEncoderRatio * kShooterEncoderPPR; // based on gear reduction between encoder and output shaft, and encoder ppr
    public static final double kShooterAllowableErrorRPM = 100.0; // TODO Tune for 2022 Upper Hub
        
        // Speeds
        public static final double kShooterIdleRPM = 0;
        public static final double kShooterIdleRPMAuto = 2000;
        public static final double kShooterFenderRPM = 2150;
        public static final double kShooterProtectedRPM = 2700;
        public static final double kShooterTerminalRPM = 3150; 

    // Hood
    public static final double kHoodHomePositionDegress = 55;
    public static final double kHoodMaxPositionDegress = 55;
    public static final double kHoodMinPositionDegress = 1;
    public static final double kHoodKp = 0.5;
    public static final double kHoodKi = 0.008;
    public static final double kHoodKd = 0.02;
    public static final double kHoodKf = 0.045;
    public static final double kHoodEncoderPPR = 4096.0;
    public static final double kHoodMotionCruiseVelocity = 6000;
    public static final double kHoodMotionAcceleration = 12000;
    public static final double kHoodOutputToEncoderRatio = 392.0/18.0;
    public static final double kHoodTicksPerRevolution = kHoodOutputToEncoderRatio * kHoodEncoderPPR; // based on gear reduction between encoder and output shaft, and encoder ppr
    public static final double kHoodTicksPerDegree = kHoodTicksPerRevolution / 360.0;
    public static final double kHoodPositionDeadband = 1; // TODO Tune for 2022 Upper Hub

        // Degrees
        public static final double kHoodIdleDegree = 30;
        public static final double kHoodFenderDegree = 5;
        public static final double kHoodProtectedDegree = 33;
        public static final double kHoodTerminalDegree = 42;

    // Climb
    public static final double kClimbHomePositionInches = 45.5;
    public static final double kClimbMaxPositionInches = 66;
    public static final double kClimbMinPositionInches = 45.5;
    public static final double kClimbHomePositionTicks = 0;
    public static final double kClimbMaxPositionTicks = 53000;
    public static final double kClimbMinPositionTicks = 308;
    public static final double kClimbKp = 0.5;
    public static final double kClimbKi = 0.008;
    public static final double kClimbKd = 0.02;
    public static final double kClimbKf = 0.045;
    public static final double kClimbEncoderPPR = 2048.0;
    public static final double kClimbMotionCruiseVelocity = 6000;
    public static final double kClimbMotionAcceleration = 12000;
    public static final double kClimbOutputToEncoderRatio = 36.0/2.0;
    public static final double kClimbTicksPerRevolution = kClimbOutputToEncoderRatio * kClimbEncoderPPR; // based on gear reduction between encoder and output shaft, and encoder ppr
    public static final double kClimbPositionDeadband = 1; 

        // Heights
        public static final double kClimbMidInches = 61;
        public static final double kClimbMidTicks = 38000;

        public static final double kClimbRetractedInches = 50;
        public static final double kClimbRetractedTicks = 10000;

    // Limelight TODO change values for 2022 bot
    public static final double kGoalMaxHeightInches = 104; // Height from carpet to top of upper hub 
    public static final double kLimelightAngleHorizontalPlaneToLens = 27.5; // Rotation Angle
    public static final double kLimelightLensOffGroundHeight = 43.0625;        // Measurment from carpet to center or LL lens

    // Interpolation Map
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLobHoodMap = new InterpolatingTreeMap<>();
    static {
        kLobHoodMap.put(new InterpolatingDouble(56.0), new InterpolatingDouble(3.0));
        kLobHoodMap.put(new InterpolatingDouble(87.0), new InterpolatingDouble(14.0));
        kLobHoodMap.put(new InterpolatingDouble(107.0), new InterpolatingDouble(21.5)); //23
        kLobHoodMap.put(new InterpolatingDouble(135.0), new InterpolatingDouble(29.0));
        kLobHoodMap.put(new InterpolatingDouble(187.0), new InterpolatingDouble(33.0)); //33
        kLobHoodMap.put(new InterpolatingDouble(232.0), new InterpolatingDouble(37.0));
        kLobHoodMap.put(new InterpolatingDouble(262.0), new InterpolatingDouble(41.0));
        kLobHoodMap.put(new InterpolatingDouble(300.0), new InterpolatingDouble(42.0));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLobRPMMap = new InterpolatingTreeMap<>();
    static {

        kLobRPMMap.put(new InterpolatingDouble(56.0), new InterpolatingDouble(2150.0));
        kLobRPMMap.put(new InterpolatingDouble(87.0), new InterpolatingDouble(2250.0));
        kLobRPMMap.put(new InterpolatingDouble(107.0), new InterpolatingDouble(2360.0));
        kLobRPMMap.put(new InterpolatingDouble(135.0), new InterpolatingDouble(2450.0));
        kLobRPMMap.put(new InterpolatingDouble(187.0), new InterpolatingDouble(2700.0)); //2750
        kLobRPMMap.put(new InterpolatingDouble(232.0), new InterpolatingDouble(2900.0)); //2950
        kLobRPMMap.put(new InterpolatingDouble(262.0), new InterpolatingDouble(3100.0)); 
        kLobRPMMap.put(new InterpolatingDouble(300.0), new InterpolatingDouble(3275.0));

    }
}
