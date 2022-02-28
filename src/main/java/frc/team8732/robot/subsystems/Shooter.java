// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team8732.lib.drivers.TalonSRXFactory;
import frc.team8732.lib.drivers.TalonUtil;
import frc.team8732.lib.util.ReflectingCSVWriter;
import frc.team8732.lib.util.Util;
import frc.team8732.robot.Constants;
import frc.team8732.robot.loops.ILooper;
import frc.team8732.robot.loops.Loop;

/** This is the Shooter subsystem for the 2022 robot. It contains all the information 
 * (variables, constructors, methods etc.) in order to allow the shooter to function.*/
public class Shooter extends Subsystem {
    private static Shooter mInstance;

    public synchronized static Shooter getInstance() {
        if(mInstance == null){
            mInstance = new Shooter();
        }

        return mInstance;
    }

    // Hardware
    private final TalonSRX mShooterMaster, mShooterSlave;

    // Control States
    private ShooterControlState mShooterControlState;
    private ShooterState mShooterState;

    public enum ShooterControlState {
        OPEN_LOOP,  // open loop voltage control 
        VELOCITY    // velocity control
    }

    public enum ShooterState {
        OFF,      
        PERCENT_OUTPUT,  
        IDLE,           // Fixed shooter rate > 0 rpm to allow for quick shooter ramp
        RAW_RPM,        // Set shooter rpm for known locations (not factoring in LL distance)
        CALCULATED_RPM  // Set shooter rpm for unknown locations (uses LL + TreeMap to get ideal motor demand)
    }

    // Hardware States
    private final int kPIDSlot = 0;

    public static class PeriodicIO {
        // MOTOR OUTPUT
        public double demand = 0.0;

        // INPUTS
        public double timestamp;
        public double output_voltage;
        public double supply_current;
        public double stator_current;
        public double velocity_ticks_per_100_ms = 0.0; // Talon SRX + 775 Pro
        public double velocity_rpm = 0.0;              // Talon SRX + 775 Pro
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private void configureMaster(TalonSRX talon){
        // General
        talon.setInverted(InvertType.None);

        // Encoder 
        TalonUtil.checkError(mShooterMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDSlot,
                Constants.kLongCANTimeoutMs), "Shooter Master: Could not detect encoder: ");

        // PID
        TalonUtil.checkError(talon.config_kP(kPIDSlot, Constants.kShooterKp, Constants.kLongCANTimeoutMs), "Could not set shooter velocity kp");
        TalonUtil.checkError(talon.config_kI(kPIDSlot, Constants.kShooterKi, Constants.kLongCANTimeoutMs), "Could not set shooter velocity ki");
        TalonUtil.checkError(talon.config_kD(kPIDSlot, Constants.kShooterKd, Constants.kLongCANTimeoutMs), "Could not set shooter velocity kd");
        TalonUtil.checkError(talon.config_kF(kPIDSlot, Constants.kShooterKf, Constants.kLongCANTimeoutMs), "Could not set shooter velocity kf");

        // Sensor Velocity Measure
        TalonUtil.checkError(talon.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs), "Could not config shooter velocity measurement period");
        TalonUtil.checkError(talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs), "Could not config shooter velocity measurement window");

        // Voltage Comp
        TalonUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config shooter voltage comp saturation");
        talon.enableVoltageCompensation(true);
    }

    private Shooter() {
        mPeriodicIO = new PeriodicIO();

        mShooterMaster = TalonSRXFactory.createDefaultTalon(Constants.kShooterMasterID);
        configureMaster(mShooterMaster);

        mShooterSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kShooterSlaveID, Constants.kShooterMasterID);
        mShooterSlave.setInverted(InvertType.FollowMaster);
    }

    public synchronized ShooterControlState getControlState() {
        return mShooterControlState;
    }

    public synchronized void setControlState(ShooterControlState controlState) {
        this.mShooterControlState = controlState;
    }

    public synchronized ShooterState getShooterState(){
        return mShooterState;
    }

    public synchronized void setShooterState(ShooterState shooterState) {
        this.mShooterState = shooterState;
    }

    // Subsystem looper methods 
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.output_voltage = mShooterMaster.getMotorOutputVoltage();
        mPeriodicIO.supply_current = mShooterMaster.getSupplyCurrent();
        mPeriodicIO.stator_current = mShooterMaster.getStatorCurrent();

        mPeriodicIO.velocity_ticks_per_100_ms = mShooterMaster.getSelectedSensorVelocity(0);

        mPeriodicIO.velocity_rpm = nativeUnitsToRPM(mPeriodicIO.velocity_ticks_per_100_ms);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (mShooterControlState == ShooterControlState.OPEN_LOOP) {
            mShooterMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
        } else if (mShooterControlState == ShooterControlState.VELOCITY) {
            mShooterMaster.set(ControlMode.Velocity, mPeriodicIO.demand);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                synchronized (Shooter.this) {
                    switch (mShooterState) {
                        case OFF:
                            // No looping calculations needed
                            break;
                        case PERCENT_OUTPUT:
                            // No looping calculations needed
                            break;
                        case IDLE:
                            // No looping calculations needed
                            break;
                        case RAW_RPM:
                            // No looping calculations needed
                            break;
                        case CALCULATED_RPM:
                            // calculateDesiredRPM (Via LL distance + Interpolating Map values) (Need LL subsystem done)
                            break;
                        default:
                            System.out.println("unexpected shooter state: " + getShooterState());
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    // Shooter conversion methdos
    /**
     * @param ticks per 100 ms
     * @return rpm
     */
    public double nativeUnitsToRPM(double ticks_per_100_ms) {
        return ticks_per_100_ms * 10.0 * 60.0 / Constants.kShooterTicksPerRevolution;
    }

    /**
     * @param rpm
     * @return ticks per 100 ms
     */
    public double rpmToNativeUnits(double rpm) {
        return rpm / 60.0 / 10.0 * Constants.kShooterTicksPerRevolution;
    }

    // Shooter accessor methods
    public synchronized double getRPM() {
        return nativeUnitsToRPM(getVelocityNativeUnits());
    }

    public synchronized double getVelocityNativeUnits() {
        return mPeriodicIO.velocity_ticks_per_100_ms;
    }

    public synchronized double getDemandRPM() {
        return nativeUnitsToRPM(mPeriodicIO.demand);
    }

    // Shooter modifier methods 
    public synchronized void setOpenLoop(double power) {
        if (mShooterControlState != ShooterControlState.OPEN_LOOP) {
            mShooterControlState = ShooterControlState.OPEN_LOOP;
        }

        mPeriodicIO.demand = power;
    }

    public synchronized void setRPM(double rpm) {
        if (mShooterControlState != ShooterControlState.VELOCITY) {
            mShooterControlState = ShooterControlState.VELOCITY;
        }

        mPeriodicIO.demand = rpmToNativeUnits(rpm);
    }

    public synchronized boolean isAtSetpoint() {
        return Util.epsilonEquals(getRPM(), getDemandRPM(),
                Constants.kShooterAllowableErrorRPM);
    }

    @Override
    public void stop() {
        mShooterMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    // CSV Logging
    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/SHOOTER-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter Master RPM", getRPM());
        SmartDashboard.putNumber("Shooter Demand", mShooterControlState == ShooterControlState.OPEN_LOOP ? mPeriodicIO.demand
                : (mShooterControlState == ShooterControlState.VELOCITY ? nativeUnitsToRPM(mPeriodicIO.demand) : 0.0));
        SmartDashboard.putBoolean("Shooter At Setpoint", isAtSetpoint());

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }
}