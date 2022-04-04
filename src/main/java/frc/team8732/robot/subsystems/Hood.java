// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team8732.lib.drivers.TalonSRXFactory;
import frc.team8732.lib.drivers.TalonUtil;
import frc.team8732.lib.util.InterpolatingDouble;
import frc.team8732.lib.util.ReflectingCSVWriter;
import frc.team8732.lib.util.Util;
import frc.team8732.robot.Constants;
import frc.team8732.robot.Limelight;
import frc.team8732.robot.loops.ILooper;
import frc.team8732.robot.loops.Loop;

/** This is the Hood subsystem for the 2022 robot. It contains all the information 
 * (variables, constructors, methods etc.) in order to allow the hood to function.*/
public class Hood extends Subsystem {
    private static Hood mInstance;

    public synchronized static Hood getInstance() {
        if(mInstance == null){
            mInstance = new Hood();
        }

        return mInstance;
    }

    // Hardware
    private final TalonSRX mHoodMaster;

    // Control States
    private HoodControlState mHoodControlState = HoodControlState.OPEN_LOOP;

    public enum HoodControlState {
        OPEN_LOOP,  // open loop voltage control 
        MOTION_MAGIC    // velocity control
    }

    // Hardware States
    private final int kPIDSlot = 0;

    public static class PeriodicIO {
        // MOTOR OUTPUT
        public double demand = 0.0; // Hood Angle

        // INPUTS
        public double timestamp;
        public double output_voltage;
        public double supply_current;
        public double stator_current;
        public double position_ticks = 0.0;     // Talon SRX + 775 Pro
        public double position_degrees = 0.0;   // Talon SRX + 775 Pro
        public double calculated_degree = 0.0;
        public double offset_degree = 0.0;
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private void configureMaster(TalonSRX talon){
        // General 
        mHoodMaster.setInverted(InvertType.InvertMotorOutput);

        // Encoder 
        TalonUtil.checkError(mHoodMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDSlot,
                Constants.kLongCANTimeoutMs), "Hood Master: Could not detect encoder: ");

        // PID
        TalonUtil.checkError(talon.config_kP(kPIDSlot, Constants.kHoodKp, Constants.kLongCANTimeoutMs), "Could not set hood position kp");
        TalonUtil.checkError(talon.config_kI(kPIDSlot, Constants.kHoodKi, Constants.kLongCANTimeoutMs), "Could not set hood position ki");
        TalonUtil.checkError(talon.config_kD(kPIDSlot, Constants.kHoodKd, Constants.kLongCANTimeoutMs), "Could not set hood position kd");
        TalonUtil.checkError(talon.config_kF(kPIDSlot, Constants.kHoodKf, Constants.kLongCANTimeoutMs), "Could not set hood position kf");

        // Sensor Velocity Measure
        TalonUtil.checkError(talon.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs), "Could not config hood velocity measurement period");
        TalonUtil.checkError(talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs), "Could not config hood velocity measurement window");

        // MM
        TalonUtil.checkError(talon.configMotionCruiseVelocity(Constants.kHoodMotionCruiseVelocity, Constants.kLongCANTimeoutMs), "Could not set hood motion cruise velocity");
        TalonUtil.checkError(talon.configMotionAcceleration(Constants.kHoodMotionAcceleration, Constants.kLongCANTimeoutMs), "Could not set hood motion acceleration");

        // Voltage Comp
        TalonUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config hood voltage comp saturation");
        talon.enableVoltageCompensation(true);
    }

    private Hood() {
        mPeriodicIO = new PeriodicIO();

        mHoodMaster = TalonSRXFactory.createDefaultTalon(Constants.kHoodMotorID);
        configureMaster(mHoodMaster);
        mHoodMaster.setSensorPhase(true);

    }

    // Subsystem looper methods 
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.output_voltage = mHoodMaster.getMotorOutputVoltage();
        mPeriodicIO.supply_current = mHoodMaster.getSupplyCurrent();
        mPeriodicIO.stator_current = mHoodMaster.getStatorCurrent();

        mPeriodicIO.position_ticks = mHoodMaster.getSelectedSensorPosition(0);

        mPeriodicIO.position_degrees = nativeUnitsToDegree(mPeriodicIO.position_ticks);
        mPeriodicIO.calculated_degree = calculatedDesiredDegree();
        mPeriodicIO.offset_degree = getOffsetDegree();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (mHoodControlState == HoodControlState.OPEN_LOOP) {
            // mHoodMaster.set(ControlMode.PercentOutput, RobotContainer.getInstance().getOperatorGameController().getLeftYAxis());
        } else if (mHoodControlState == HoodControlState.MOTION_MAGIC) {
            mHoodMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, .07);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    // Hood conversion methdos
    /**
     * @param ticks position
     * @return degree
     */
    public double nativeUnitsToDegree(double ticks) {
        return (double) (ticks / Constants.kHoodTicksPerDegree) 
        + Constants.kHoodHomePositionDegress;
    }

    /**
     * @param degree
     * @return ticks per 100 ms
     */
    public double degreeToNativeUnits(double degree) {
        return (int) (degree - Constants.kHoodHomePositionDegress) 
        * Constants.kHoodTicksPerDegree;
    }

    
    public synchronized double calculatedDesiredDegree() {
        Limelight mLimelight = Limelight.getInstance();
        InterpolatingDouble interpolatedDegree =  Constants.kLobHoodMap.getInterpolated(new InterpolatingDouble(mLimelight.getLimelightDistanceFromTarget()));
        return interpolatedDegree.value;
    }

    // Hood accessor methods
    public synchronized double getPositionNativeUnits() {
        return mPeriodicIO.position_ticks;
    }

    public synchronized double getDegree() {
        return nativeUnitsToDegree(getPositionNativeUnits());
    }

    public synchronized double getDemandDegree() {
        return nativeUnitsToDegree(mPeriodicIO.demand);
    }

    public synchronized double getCalculatedDegree() {
        return mPeriodicIO.calculated_degree;
    }

    public synchronized double getOffsetDegree() {
        return mPeriodicIO.offset_degree;
    }

    // Hood modifier methods 
    public synchronized void setOpenLoop(double power) {
        if (mHoodControlState != HoodControlState.OPEN_LOOP) {
            mHoodControlState = HoodControlState.OPEN_LOOP;
        }

        mPeriodicIO.demand = power;
    }

    public synchronized void setOffsetDegree(double offset) {
        mPeriodicIO.offset_degree = mPeriodicIO.offset_degree + offset;
    }

    private double limitHoodAngle(double targetDegree) {
        if (targetDegree < Constants.kHoodMinPositionDegress) {
            return Constants.kHoodMinPositionDegress;
        } else if (targetDegree > Constants.kHoodMaxPositionDegress) {
            return Constants.kHoodMaxPositionDegress;
        }

        return targetDegree;
    }

    public synchronized void setDegree(double degree) {
        if (mHoodControlState != HoodControlState.MOTION_MAGIC) {
            mHoodControlState = HoodControlState.MOTION_MAGIC;
        }

        double filteredDegree = limitHoodAngle(degree);
        mPeriodicIO.demand = degreeToNativeUnits(filteredDegree);
    }

    public synchronized boolean isAtSetpoint() {
        return Util.epsilonEquals(getDegree(), getDemandDegree(),
                Constants.kHoodPositionDeadband);
    }

    @Override
    public void stop() {
        mHoodMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    // CSV Logging
    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/HOOD-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
    public double angle;
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Master Degree", getDegree());
        SmartDashboard.putNumber("Hood Demand", mHoodControlState == HoodControlState.OPEN_LOOP ? mPeriodicIO.demand
                : (mHoodControlState == HoodControlState.MOTION_MAGIC ? nativeUnitsToDegree(mPeriodicIO.demand) : 0.0));
        SmartDashboard.putBoolean("Hood At Setpoint", isAtSetpoint());
        SmartDashboard.putNumber("Calculated Degree", getCalculatedDegree());
        SmartDashboard.putNumber("Offset Degree", getOffsetDegree());

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }
}