// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team8732.lib.drivers.TalonFXFactory;
import frc.team8732.lib.drivers.TalonUtil;
import frc.team8732.lib.util.ReflectingCSVWriter;
import frc.team8732.lib.util.Util;
import frc.team8732.robot.Constants;
import frc.team8732.robot.loops.ILooper;
import frc.team8732.robot.loops.Loop;

/** This is the Climb subsystem for the 2022 robot. It contains all the information 
 * (variables, constructors, methods etc.) in order to allow the climb to function.*/
public class Climb extends Subsystem {
    private static Climb mInstance;

    public synchronized static Climb getInstance() {
        if(mInstance == null){
            mInstance = new Climb();
        }

        return mInstance;
    }

    // Hardware
    private final TalonFX mClimbMaster, mClimbSlave;

    // Control States
    private ClimbControlState mClimbControlState = ClimbControlState.OPEN_LOOP;

    public enum ClimbControlState {
        OPEN_LOOP,  // open loop voltage control 
        MOTION_MAGIC    // velocity control
    }

    // Hardware States
    private final int kPIDSlot = 0;

    public static class PeriodicIO {
        // MOTOR OUTPUT
        public double demand = 0.0; // Climber Height

        // INPUTS
        public double timestamp;
        public double output_voltage;
        public double supply_current;
        public double stator_current;
        public double position_ticks = 0.0;     // Falcon FX
        public double position_inches = 0.0;   // Falcon FX
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private void configureMaster(TalonFX talon){
        // General 
        mClimbMaster.setInverted(InvertType.None);

        // Encoder 
        TalonUtil.checkError(mClimbMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDSlot,
                Constants.kLongCANTimeoutMs), "Climb Master: Could not detect encoder: ");

        // PID
        TalonUtil.checkError(talon.config_kP(kPIDSlot, Constants.kClimbKp, Constants.kLongCANTimeoutMs), "Could not set climb position kp");
        TalonUtil.checkError(talon.config_kI(kPIDSlot, Constants.kClimbKi, Constants.kLongCANTimeoutMs), "Could not set climb position ki");
        TalonUtil.checkError(talon.config_kD(kPIDSlot, Constants.kClimbKd, Constants.kLongCANTimeoutMs), "Could not set climb position kd");
        TalonUtil.checkError(talon.config_kF(kPIDSlot, Constants.kClimbKf, Constants.kLongCANTimeoutMs), "Could not set climb position kf");

        // Sensor Velocity Measure
        TalonUtil.checkError(talon.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs), "Could not config climb velocity measurement period");
        TalonUtil.checkError(talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs), "Could not config climb velocity measurement window");

        // MM
        TalonUtil.checkError(talon.configMotionCruiseVelocity(Constants.kClimbMotionCruiseVelocity, Constants.kLongCANTimeoutMs), "Could not set climb motion cruise velocity");
        TalonUtil.checkError(talon.configMotionAcceleration(Constants.kClimbMotionAcceleration, Constants.kLongCANTimeoutMs), "Could not set climb motion acceleration");

        // Voltage Comp
        TalonUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config climb voltage comp saturation");
        talon.enableVoltageCompensation(true);
    }

    private Climb() {
        mPeriodicIO = new PeriodicIO();

        mClimbMaster = TalonFXFactory.createDefaultTalon(Constants.kClimbMasterID);
        configureMaster(mClimbMaster);

        mClimbSlave = TalonFXFactory.createPermanentSlaveTalon(Constants.kClimbSlaveID, Constants.kClimbMasterID);
        mClimbSlave.setInverted(InvertType.InvertMotorOutput);

    }

    // Subsystem looper methods 
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.output_voltage = mClimbMaster.getMotorOutputVoltage();
        mPeriodicIO.supply_current = mClimbMaster.getSupplyCurrent();
        mPeriodicIO.stator_current = mClimbMaster.getStatorCurrent();

        mPeriodicIO.position_ticks = mClimbMaster.getSelectedSensorPosition(0);

        mPeriodicIO.position_inches = nativeUnitsToInches(mPeriodicIO.position_ticks);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (mClimbControlState == ClimbControlState.OPEN_LOOP) {
            mClimbMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
            mClimbMaster.setNeutralMode(NeutralMode.Brake);
            mClimbSlave.setNeutralMode(NeutralMode.Brake);

        } else if (mClimbControlState == ClimbControlState.MOTION_MAGIC) {
            mClimbMaster.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, .07);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {zeroSensors();}

            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    // Climb conversion methdos
    /**
     * @param ticks position
     * @return inches
     */
    public double nativeUnitsToInches(double ticks) {
        return (double) (ticks / Constants.kClimbTicksPerRevolution) 
        + Constants.kClimbHomePositionInches;
    }

    /**
     * @param inches
     * @return ticks per 100 ms
     */
    public double inchesToNativeUnits(double inches) {
        return (int) (inches - Constants.kClimbHomePositionInches) 
        * Constants.kClimbTicksPerRevolution;
    }

    // Climb accessor methods
    public synchronized double getPositionNativeUnits() {
        return mPeriodicIO.position_ticks;
    }

    public synchronized double getInches() {
        return nativeUnitsToInches(getPositionNativeUnits());
    }

    public synchronized double getDemandInches() {
        return nativeUnitsToInches(mPeriodicIO.demand);
    }

    // Climb modifier methods 
    public synchronized void setOpenLoop(double power) {
        if (mClimbControlState != ClimbControlState.OPEN_LOOP) {
            mClimbControlState = ClimbControlState.OPEN_LOOP;
        }

        mPeriodicIO.demand = power;
    }

    // private double limitClimbInches(double targetInches) {
    //     if (targetInches < Constants.kClimbMinPositionInches) {
    //         return Constants.kClimbMinPositionInches;
    //     } else if (targetInches > Constants.kClimbMaxPositionInches) {
    //         return Constants.kClimbMaxPositionInches;
    //     }

    //     return targetInches;
    // }

    // public synchronized void setInches(double inches) {
    //     if (mClimbControlState != ClimbControlState.MOTION_MAGIC) {
    //         mClimbControlState = ClimbControlState.MOTION_MAGIC;
    //     }

    //     double filterdInches = limitClimbInches(inches);
    //     mPeriodicIO.demand = inchesToNativeUnits(filterdInches);
    // }

    private double limitClimbTicks(double targetTicks) {
        if (targetTicks < Constants.kClimbMinPositionTicks) {
            return Constants.kClimbMinPositionTicks;
        } else if (targetTicks > Constants.kClimbMaxPositionTicks) {
            return Constants.kClimbMaxPositionTicks;
        }

        return targetTicks;
    }

    public synchronized void setTicks(double ticks) {
        if (mClimbControlState != ClimbControlState.MOTION_MAGIC) {
            mClimbControlState = ClimbControlState.MOTION_MAGIC;
        }

        double filterdTicks = limitClimbTicks(ticks);
        mPeriodicIO.demand = filterdTicks;
    }

    public synchronized boolean isAtSetpoint() {
        return Util.epsilonEquals(getPositionNativeUnits(), mPeriodicIO.demand,
                2048);
    }

    @Override
    public void stop() {
        mClimbMaster.set(ControlMode.PercentOutput, 0.0);
        zeroSensors();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void zeroSensors(){
        mPeriodicIO.position_ticks = 0;
    }

    // CSV Logging
    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/CLIMB-LOGS.csv", PeriodicIO.class);
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
        SmartDashboard.putNumber("Climb Master Ticks", getPositionNativeUnits());
        SmartDashboard.putNumber("Climb Demand", mClimbControlState == ClimbControlState.OPEN_LOOP ? mPeriodicIO.demand
                : (mClimbControlState == ClimbControlState.MOTION_MAGIC ? nativeUnitsToInches(mPeriodicIO.demand) : 0.0));
        SmartDashboard.putBoolean("Climb At Setpoint", isAtSetpoint());

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }
}