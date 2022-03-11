package frc.team8732.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team8732.lib.drivers.MotorChecker;
import frc.team8732.lib.drivers.TalonChecker;
import frc.team8732.lib.drivers.TalonSRXFactory;
import frc.team8732.lib.drivers.TalonUtil;
import frc.team8732.lib.geometry.Pose2d;
import frc.team8732.lib.geometry.Pose2dWithCurvature;
import frc.team8732.lib.geometry.Rotation2d;
import frc.team8732.lib.trajectory.TrajectoryIterator;
import frc.team8732.lib.trajectory.timing.TimedState;
import frc.team8732.lib.util.DriveOutput;
import frc.team8732.lib.util.DriveSignal;
import frc.team8732.lib.util.ReflectingCSVWriter;
import frc.team8732.robot.Constants;
import frc.team8732.robot.RobotContainer;
import frc.team8732.robot.RobotState;
import frc.team8732.robot.controller.GameController;
import frc.team8732.robot.loops.ILooper;
import frc.team8732.robot.loops.Loop;
import frc.team8732.robot.planners.DriveMotionPlanner;

/** This is the Drive subsystem for the 2022 robot. It contains all the information 
 * (variables, constructors, methods etc.) in order to allow the drivetrain to function.*/
public class Drive extends Subsystem {
    private static Drive mInstance;

    public synchronized static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;

    // Control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control,
        VELOCITY, // velocity control
        PATH_FOLLOWING,
        JOYSTICK
    }

    // Hardware states
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();

    private DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    private final int kPIDSlot = 0;

    public static class PeriodicIO {
        // MOTOR OUTPUT
        public double left_demand;
        public double right_demand;

        // INPUTS
        public double timestamp;
        public double left_voltage;
        public double right_voltage;
        public double left_position_ticks; // Using Mag Encoder
        public double right_position_ticks; // Using Mag Encoder
        public double left_distance;
        public double right_distance;
        public double left_velocity_ticks_per_100ms; // Talon SRX + MiniCIM
        public double right_velocity_ticks_per_100ms; // Talon SRX + MiniCIM
        public double left_velocity_in_per_sec;
        public double right_velocity_in_per_sec;
        public Rotation2d gyro_heading = Rotation2d.identity();

        // OBSERVED OUTPUTS
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public Pose2d error = Pose2d.identity();
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private void configureMaster(TalonSRX talon, boolean left) {
        // General
        TalonUtil.checkError(talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kLongCANTimeoutMs), "Could not set talon status frame period");
        talon.setInverted(!left);
        talon.setSensorPhase(true);
        TalonUtil.checkError(talon.configForwardSoftLimitEnable(false), "Could not set forward soft limit");
        TalonUtil.checkError(talon.configReverseSoftLimitEnable(false), "Could not set reverse soft limit");
        talon.configNeutralDeadband(0.02, 0);


        // Encoder 
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kLongCANTimeoutMs); //primary closed-loop, 100 ms timeout

        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }

        // PID
        TalonUtil.checkError(talon.config_kP(kPIDSlot, Constants.kDriveKp, Constants.kLongCANTimeoutMs), "Could not set drive velocity kp");
        TalonUtil.checkError(talon.config_kI(kPIDSlot, Constants.kDriveKi, Constants.kLongCANTimeoutMs), "Could not set drive velocity ki");
        TalonUtil.checkError(talon.config_kD(kPIDSlot, Constants.kDriveKd, Constants.kLongCANTimeoutMs), "Could not set velocity kd");
        TalonUtil.checkError(talon.config_kF(kPIDSlot, Constants.kDriveKf, Constants.kLongCANTimeoutMs), "Could not set velocity kf");

        int kPositionPIDSlot = 1;
        TalonUtil.checkError(talon.config_kP(kPositionPIDSlot, Constants.kDrivePositionKp, Constants.kLongCANTimeoutMs), "Could not set drive position kp");
        TalonUtil.checkError(talon.config_kI(kPositionPIDSlot, Constants.kDrivePositionKi, Constants.kLongCANTimeoutMs), "Could not set drive position ki");
        TalonUtil.checkError(talon.config_kD(kPositionPIDSlot, Constants.kDrivePositionKd, Constants.kLongCANTimeoutMs), "Could not set drive position kd");
        TalonUtil.checkError(talon.config_kF(kPositionPIDSlot, Constants.kDrivePositionKf, Constants.kLongCANTimeoutMs), "Could not set drive position kf");

        // Sensor Velocity Measure
        TalonUtil.checkError(talon.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs), "Could not config drive velocity measurement period");
        TalonUtil.checkError(talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs), "Could not config drive velocity measurement window");

        // Voltage Comp
        TalonUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config drive voltage comp saturation");
        talon.enableVoltageCompensation(true);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // Start all Talons in open loop mode
        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kLeftDriveMasterID);
        configureMaster(mLeftMaster, true);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveAID,
                Constants.kLeftDriveMasterID);
        mLeftSlaveA.setInverted(false);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveBID,
                Constants.kLeftDriveMasterID);
        mLeftSlaveB.setInverted(false);

        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kRightDriveMasterID);
        configureMaster(mRightMaster, false);

        mRightSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveAID,
                Constants.kRightDriveMasterID);
        mRightSlaveA.setInverted(true);

        mRightSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveBID,
                Constants.kRightDriveMasterID);
        mRightSlaveB.setInverted(true);

        mPigeon = new PigeonIMU(mRightSlaveA);
        mRightSlaveA.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();

        zeroSensors();
    }

    public synchronized DriveControlState getControlState() {
        return mDriveControlState;
    }

    public synchronized void setControlState(DriveControlState controlState) {
        this.mDriveControlState = controlState;
    }

    // Subsystem looper methods 
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;

        mPeriodicIO.left_voltage = mLeftMaster.getMotorOutputVoltage();
        mPeriodicIO.right_voltage = mRightMaster.getMotorOutputVoltage();

        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);

        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / Constants.kDriveEncoderPPR) * Math.PI;
        mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / Constants.kDriveEncoderPPR) * Math.PI;
        mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);

        mPeriodicIO.left_velocity_in_per_sec = getLeftLinearVelocity();
        mPeriodicIO.right_velocity_in_per_sec = getRightLinearVelocity();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
        } else if (mDriveControlState == DriveControlState.VELOCITY || mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            double kd = Constants.kDriveKd;
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + kd * mPeriodicIO.left_accel / 1023.0);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + kd * mPeriodicIO.right_accel / 1023.0);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    setBrakeMode(true);
                    // startLogging();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case VELOCITY:
                            break;
                        case PATH_FOLLOWING:
                            updatePathFollower();
                            break;
                        case JOYSTICK:
                            driveWithJoystick();
                            break;
                        default:
                            System.out.println("unexpected drive control state: " + getControlState());
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                // stopLogging();
            }
        });
    }

    // Drive conversions methods
    public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesToRadians(double inches) {
        return inches * 2.0 / Constants.kDriveWheelDiameterInches;
    }

    public static double radiansToInches(double radians) {
        return radians / 2.0 * Constants.kDriveWheelDiameterInches;
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    /**
     * @param rad_s of the output
     * @return ticks per 100 ms of the talon srx mag encoder
     */
    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * Constants.kDriveEncoderPPR / 10.0;
    }

    // Drive accessor methods
    public double getLeftEncoderDistance() {
        return mPeriodicIO.left_distance;
    }

    public double getRightEncoderDistance() {
        return mPeriodicIO.right_distance;
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / Constants.kDriveEncoderPPR);
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / Constants.kDriveEncoderPPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return (Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity())) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public double getLeftOutputVoltage() {
        return mPeriodicIO.left_voltage;
    }

    public double getRightOutputVoltage() {
        return mPeriodicIO.right_voltage;
    }

    public double getAverageOutputVoltageMagnitude() {
        return (Math.abs(getLeftOutputVoltage()) + Math.abs(getRightOutputVoltage())) / 2.0;
    }

    // Drive modifier methods
    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    // Path Following
    /**
     * Configure talons for following via the ramsete controller
     */
    public synchronized void setRamseteVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            configureTalonPIDSlot();
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveOutput output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setRamseteVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setRamseteVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized void configureTalonPIDSlot() {
        int desired_slot_idx = kPIDSlot;

        mLeftMaster.selectProfileSlot(desired_slot_idx, 0);
        mRightMaster.selectProfileSlot(desired_slot_idx, 0);
    }

    public synchronized void driveWithJoystick() {
        GameController driverController = RobotContainer.getInstance().getDriveGameController();

        double throttleScalar = .50;
        double wheelScalar = .20;

        double throttle = (-1 * driverController.getLeftYAxis()) * throttleScalar; 
        double wheel = (driverController.getRightXAxis() * wheelScalar);
    
        mPeriodicIO.left_demand = wheel + throttle ;
        mPeriodicIO.right_demand = throttle - wheel;

        mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
        mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
    }

    // Brake State
    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            NeutralMode mode = shouldEnable ? NeutralMode.Brake : NeutralMode.Coast;

            mLeftMaster.setNeutralMode(mode);
            mRightMaster.setNeutralMode(mode);
        }
    }

    // Sensor State
    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        mRightMaster.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);

        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    // CSV Logging
    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    @Override
    public boolean checkSystem() {
        // Trigger write to TalonSRXs.
        stop();
        writePeriodicOutputs();

        setBrakeMode(false);

        boolean leftSide = TalonChecker.checkMotors(this,
                new ArrayList<>() {
                    {
                        add(new MotorChecker.MotorConfig<>("left_master", mLeftMaster));
                        add(new MotorChecker.MotorConfig<>("left_slave_A", mLeftSlaveA));
                        add(new MotorChecker.MotorConfig<>("left_slave_B", mLeftSlaveB));
                    }
                }, new MotorChecker.CheckerConfig() {
                    {
                        mCurrentFloor = 2;
                        mRPMFloor = 1500;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 250;
                        mRPMSupplier = () -> mLeftMaster.getSelectedSensorVelocity(0);
                    }
                });
        boolean rightSide = TalonChecker.checkMotors(this,
                new ArrayList<>() {
                    {
                        add(new MotorChecker.MotorConfig<>("right_master", mRightMaster));
                        add(new MotorChecker.MotorConfig<>("right_master_2", mRightSlaveA));
                        add(new MotorChecker.MotorConfig<>("right_master_3", mRightSlaveB));

                    }
                }, new MotorChecker.CheckerConfig() {
                    {
                        mCurrentFloor = 2;
                        mRPMFloor = 1500;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 250;
                        mRPMSupplier = () -> mRightMaster.getSelectedSensorVelocity(0);
                    }
                });

        return leftSide && rightSide;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        SmartDashboard.putNumber("Left Drive 1 Current", mLeftMaster.getStatorCurrent());
        SmartDashboard.putNumber("Left Drive 2 Current", mLeftSlaveA.getStatorCurrent());
        SmartDashboard.putNumber("Left Drive 3 Current", mLeftSlaveB.getStatorCurrent());
        SmartDashboard.putNumber("Right Drive 1 Current", mRightMaster.getStatorCurrent());
        SmartDashboard.putNumber("Right Drive 2 Current", mRightSlaveA.getStatorCurrent());
        SmartDashboard.putNumber("Right Drive 3 Current", mRightSlaveB.getStatorCurrent());

        SmartDashboard.putNumber("Left Drive Demand", mPeriodicIO.left_demand);
        SmartDashboard.putNumber("Right Drive Demand", mPeriodicIO.right_demand);
        SmartDashboard.putNumber("Left Drive Feedforward", mPeriodicIO.left_feedforward);
        SmartDashboard.putNumber("Right Drive Feedforward", mPeriodicIO.right_feedforward);

        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

}

   

