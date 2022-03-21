// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.team8732.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import frc.team8732.lib.drivers.TalonUtil;
import frc.team8732.lib.util.ReflectingCSVWriter;
import frc.team8732.robot.Constants;
import frc.team8732.robot.loops.ILooper;
import frc.team8732.robot.loops.Loop;
 
/** This is the Intake subsystem for the 2022 robot. It contains all the information 
 * (variables, constructors, methods etc.) in order to allow the shooter to function.*/
public class Climb extends Subsystem {
  private static Climb mInstance; 

    public synchronized static Climb getInstance() {
        if (mInstance == null){
            mInstance = new Climb();
        }
        return mInstance;
    }

    //Hardware
    private final TalonFX mClimbMotor; 

    // Control States
    private ClimbSystemState mClimbSystemState = ClimbSystemState.IDLE;

    // Motor Outputs

    public enum ClimbSystemState {
        IDLE, 
        UP,
        DOWN,
    }
    
      public synchronized void setSystemState(ClimbSystemState systemState) {
        this.mClimbSystemState = systemState;
      }

      public synchronized ClimbSystemState getSystemState() {
        return mClimbSystemState;
    }

    public static class PeriodicIO {
        // MOTOR OUTPUT
        public double demand = 0.0;

        // INPUTS
        public double timestamp;
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private Climb(){
        mPeriodicIO = new PeriodicIO();

        mClimbMotor = new TalonFX(Constants.kClimbMotorID);
        TalonUtil.checkError(mClimbMotor.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config climb voltage comp saturation");
        mClimbMotor.enableVoltageCompensation(true);
    }

    // Subsystem looper methods 
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mClimbMotor.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {setSystemState(ClimbSystemState.IDLE);}

            @Override
            public void onLoop(double timestamp) {
                switch (mClimbSystemState) {
                    case IDLE:
                        mPeriodicIO.demand = 0;
                        break;
                    case UP:
                        mPeriodicIO.demand = -1;
                        break;
                    case DOWN:
                        mPeriodicIO.demand = 1;
                    default:
                        System.out.println("Unexpected intake system state: " + mClimbSystemState);
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
        mClimbMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
    @Override
    public boolean checkSystem(){
        return false;  
    }
    @Override
    public void outputTelemetry(){
    }
}