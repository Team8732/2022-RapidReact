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
    public static final double kLooperDt = 0.01;

    // CAN
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    // Controllers
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 1;

    // Drive
    public static final int kLeftDriveMasterId = 5;
    public static final int kLeftDriveSlaveAId = 6;
    public static final int kLeftDriveSlaveBId = 7;
    public static final int kRightDriveMasterId = 12;
    public static final int kRightDriveSlaveAId = 13;
    public static final int kRightDriveSlaveBId = 14;

    // Drive ratio.
    public static final double kDriveEncoderPPR = 4096.0;
    public static final double kDriveGearReduction = 50.0 / 13.0 * 44.0 / 30.0;
    public static final double kDriveRotationsPerTick = 1.0 / kDriveEncoderPPR * 1.0 / kDriveGearReduction; // ticks * kDriveRotationsPerTickGear = wheel rotations
    public static final double kGearRatioScalar = (1.0 / (50.0 / 13.0 * 44.0 / 30.0)) / (1.0 / kDriveGearReduction); // TODO Check Scalar

    // Wheel parameters.
    public static final double kDriveWheelTrackWidthInches = 26.0;              
    public static final double kDriveWheelDiameterInches = 3.9067052758; 
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

    // Pigeon IMU
    public static final int kPigeonIMUId = 0;

}