package frc.team8732.robot;

import edu.wpi.first.math.util.Units;
import frc.team8732.lib.geometry.Translation2d;

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

    // Shooter ID's 
    public static final int kShooterMasterID = 3;
    public static final int kShooterSlaveID = 12;
    public static final int kShooterIndexerID = 4;

    // Hood
    public static final int kHoodMotorID = 11;

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
    public static final double kDriveLinearKv = 0.0438 / 2.0 * Constants.kDriveWheelDiameterInches; // V / rad/s
    public static final double kMiniCIMStallTorque = 1.41; // N*m
    public static final double kAssumedTorqueEfficiency = 0.70;
    public static final double kRobotLinearInertia = 62.051; // kg TODO
    public static final double kDriveAnalyticalLinearKa = 12.0 /* V */ / ((kDriveGearReduction * kMiniCIMStallTorque * kAssumedTorqueEfficiency * 6) / (kRobotLinearInertia * kDriveWheelRadiusMeters * kDriveWheelRadiusMeters));
    public static final double kDriveLinearKa = 0.00597 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V / rad/s^2
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
    public static final double kShooterKp = 0.0065;
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 0.0;
    public static final double kShooterKf = 0.03833127788;
    public static final double kShooterEncoderPPR = 4096.0;
    public static final double kShooterOutputToEncoderRatio = 1.0;
    public static final double kShooterTicksPerRevolution = kShooterOutputToEncoderRatio * kShooterEncoderPPR; // based on gear reduction between encoder and output shaft, and encoder ppr
    public static final double kShooterAllowableErrorRPM = 150.0; // TODO Tune for 2022 Upper Hub

    // Hood
    public static final double kHoodHomePositionDegress = 55;
    public static final double kHoodMaxPositionDegress = 55;
    public static final double kHoodMinPositionDegress = 0;
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
    public static final double kHoodPositionDeadband = 2; // TODO Tune for 2022 Upper Hub

    // Limelight TODO change values for 2022 bot
    public static final double kGoalMaxHeightInches = 104.0; // Height from carpet to top of upper hub 
    public static final double kLimelightAngleHorizontalPlaneToLens = 30.0; // Rotation Angle
    public static final double kLimelightLensOffGroundHeight = 42.0;        // Measurment from carpet to center or LL lens
}