// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.team8732.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team8732.lib.drivers.TalonSRXFactory;
import frc.team8732.robot.Constants;
 
/** This is the Intake subsystem for the 2022 robot. It contains all the information 
 * (variables, constructors, methods etc.) in order to allow the shooter to function.*/
public class Intake extends Subsystem {
  private static Intake mInstance; 
    //Hardware
    private final TalonSRX mSpicyIntake, mCactusIntake; 

        public final double mFastIntakeSpeed = .8;
        public final double mSlowIntakeSpeed = .3; 

    //get Instances
    public synchronized static Intake getInstance() {
        if (mInstance == null){
            mInstance = new Intake();
        }
        return mInstance;
    }

    public Intake(){
        mSpicyIntake = TalonSRXFactory.createDefaultTalon(Constants.kIntakeMasterID);
        mCactusIntake = TalonSRXFactory.createDefaultTalon(Constants.KIntakeSlaveID);
        }

    public void setIntakeSpeedPercent(double percentOutput){
        mSpicyIntake.set(ControlMode.PercentOutput, percentOutput);
        mCactusIntake.set(ControlMode.PercentOutput, percentOutput);
        
        }

    @Override
    public void stop(){
        mSpicyIntake.set(ControlMode.PercentOutput, 0.0);
        mCactusIntake.set(ControlMode.PercentOutput, 0.0);
    }
    
    @Override
    public boolean checkSystem(){
        return false;  
    }
    @Override
    public void outputTelemetry(){
    }
}



