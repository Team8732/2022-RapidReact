// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.team8732.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team8732.lib.drivers.TalonUtil;
import frc.team8732.lib.util.ReflectingCSVWriter;
import frc.team8732.robot.Constants;
import frc.team8732.robot.loops.ILooper;
import frc.team8732.robot.loops.Loop;
 
/** This is the Intake subsystem for the 2022 robot. It contains all the information 
 * (variables, constructors, methods etc.) in order to allow the shooter to function.*/
public class Intake extends Subsystem {
  private static Intake mInstance; 

    public synchronized static Intake getInstance() {
        if (mInstance == null){
            mInstance = new Intake();
        }
        return mInstance;
    }

    //Hardware
    private final TalonSRX mIntakeGround, mTowerIndexer, mTowerKicker; 

    private final DigitalInput mTopIndexerBeamBreak;
    private final DigitalInput mBottomIndexerBeamBreak;

    // Control States
    private IntakeSystemState mIntakeSystemState = IntakeSystemState.IDLE;

    // Motor Outputs
        // Intaking
        private double kGroundIntakeSpeed = .75;
        private double kIndexkerIntakeSpeed = .5;
        private double kKickerIntakeSpeed = .3;

        // Outtaking
        private double kGroundOuttakeSpeed = -.75;
        private double kIndexerOuttakeeSpeed = -.6;
        private double kKickerOuttakeSpeed = -.5;

        // Shooting
        private double kGroundShootingSpeed = 0;
        private double kIndexerShootingSpeed = .5;
        private double kKickerShootingSpeed = .6;

    public enum IntakeSystemState {
        IDLE, 
        INTAKING,
        OUTTAKING,
        SHOOTING
    }
    
      public synchronized void setSystemState(IntakeSystemState systemState) {
        this.mIntakeSystemState = systemState;
      }

      public synchronized IntakeSystemState getSystemState() {
        return mIntakeSystemState;
    }

    public static class PeriodicIO {
        // MOTOR OUTPUT
        public double ground_demand = 0.0;
        public double indexer_demand = 0.0;
        public double kicker_demand = 0.0;

        // INPUTS
        public double timestamp;
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private Intake(){
        mPeriodicIO = new PeriodicIO();

        mIntakeGround = new TalonSRX(Constants.kIntakeGroundID);
        TalonUtil.checkError(mIntakeGround.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config spicy intake voltage comp saturation");
        mIntakeGround.enableVoltageCompensation(true);

        mTowerIndexer = new TalonSRX(Constants.kIntakeExtensionID);
        TalonUtil.checkError(mTowerIndexer.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config cactus intake voltage comp saturation");
        mTowerIndexer.enableVoltageCompensation(true);

        mTowerKicker = new TalonSRX(Constants.kShooterIndexerID);
        TalonUtil.checkError(mTowerKicker.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config cactus intake voltage comp saturation");
        mTowerKicker.enableVoltageCompensation(true);
        mTowerKicker.setInverted(InvertType.InvertMotorOutput);

        mTopIndexerBeamBreak = new DigitalInput(Constants.kTowerTopBeamBreakID);
        mBottomIndexerBeamBreak = new DigitalInput(Constants.kTowerBottomBeamBreakID);
    }

    // Subsystem looper methods 
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        getBottomBeamBreak();
        getTopBeamBreak();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mIntakeGround.set(ControlMode.PercentOutput, mPeriodicIO.ground_demand);
        mTowerIndexer.set(ControlMode.PercentOutput, mPeriodicIO.indexer_demand);
        mTowerKicker.set(ControlMode.PercentOutput, mPeriodicIO.kicker_demand);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {setSystemState(IntakeSystemState.IDLE);}

            @Override
            public void onLoop(double timestamp) {
                switch (mIntakeSystemState) {
                    case IDLE:
                        mPeriodicIO.ground_demand = 0;
                        mPeriodicIO.indexer_demand = 0;
                        mPeriodicIO.kicker_demand = 0;
                        break;
                    case INTAKING:
                        if(getTopBeamBreak() && getBottomBeamBreak()){ // No Balls
                            mPeriodicIO.ground_demand = kGroundIntakeSpeed;
                            mPeriodicIO.indexer_demand = kIndexkerIntakeSpeed;
                            mPeriodicIO.kicker_demand = kKickerIntakeSpeed;      
                        }else if(!getTopBeamBreak() && getBottomBeamBreak()){ // One Ball @ Top Beam Break
                            mPeriodicIO.ground_demand = kGroundIntakeSpeed;
                            mPeriodicIO.indexer_demand = kIndexkerIntakeSpeed;
                            mPeriodicIO.kicker_demand = 0;     
                        }else if(getTopBeamBreak() && !getBottomBeamBreak()){ // One Ball @ Bottom Beam Break
                            mPeriodicIO.ground_demand = kGroundIntakeSpeed;
                            mPeriodicIO.indexer_demand = kIndexkerIntakeSpeed;
                            mPeriodicIO.kicker_demand = kKickerIntakeSpeed;     
                        }else if(!getTopBeamBreak() && !getBottomBeamBreak()){ // Two Balls @ Top & Bottom Beam Break
                            mPeriodicIO.ground_demand = 0;
                            mPeriodicIO.indexer_demand = 0;
                            mPeriodicIO.kicker_demand = 0;
                        }                 
                        break;
                    case OUTTAKING:
                        mPeriodicIO.ground_demand = kGroundOuttakeSpeed;
                        mPeriodicIO.indexer_demand = kIndexerOuttakeeSpeed;
                        mPeriodicIO.kicker_demand = kKickerOuttakeSpeed;                           
                        break;
                    case SHOOTING:
                        mPeriodicIO.ground_demand = kGroundShootingSpeed;
                        mPeriodicIO.indexer_demand = kIndexerShootingSpeed;
                        mPeriodicIO.kicker_demand = kKickerShootingSpeed;                           
                        break;
                    default:
                        System.out.println("Unexpected intake system state: " + mIntakeSystemState);
                        break;
            }
        }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }
    
    @Override
    public void stop(){
        mIntakeGround.set(ControlMode.PercentOutput, 0.0);
        mTowerIndexer.set(ControlMode.PercentOutput, 0.0);
        mTowerKicker.set(ControlMode.PercentOutput, 0.0);
    }

    public synchronized boolean getTopBeamBreak() {
        return mTopIndexerBeamBreak.get();
    }

    public synchronized boolean getBottomBeamBreak() {
        return mBottomIndexerBeamBreak.get();
    }
    
    @Override
    public boolean checkSystem(){
        return false;  
    }
    @Override
    public void outputTelemetry(){
        SmartDashboard.putBoolean("Top Beam Break", mTopIndexerBeamBreak.get());
        SmartDashboard.putBoolean("Bottom Beam Break", mBottomIndexerBeamBreak.get());
        SmartDashboard.putString("Intake State", mIntakeSystemState.toString());
    }
}