// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.team8732.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team8732.lib.drivers.TalonUtil;
import frc.team8732.robot.Constants;
 
/** This is the Intake subsystem for the 2022 robot. It contains all the information 
 * (variables, constructors, methods etc.) in order to allow the shooter to function.*/
public class Intake extends Subsystem {
  private static Intake mInstance; 
    //Hardware
    private final TalonSRX mIntakeGround, mIntakeExtension; 

    //get Instances
    public synchronized static Intake getInstance() {
        if (mInstance == null){
            mInstance = new Intake();
        }
        return mInstance;
    }

    public Intake(){
        mIntakeGround = new TalonSRX(Constants.kIntakeGroundID);

        TalonUtil.checkError(mIntakeGround.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config spicy intake voltage comp saturation");
        mIntakeGround.enableVoltageCompensation(true);

        mIntakeExtension = new TalonSRX(Constants.kIntakeExtensionID);

        TalonUtil.checkError(mIntakeExtension.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "Could not config cactus intake voltage comp saturation");
        mIntakeExtension.enableVoltageCompensation(true);
        }

    public void setIntakeSpeedPercent(double percentOutputGround, double percentOutputTop){
        mIntakeGround.set(ControlMode.PercentOutput, percentOutputGround);
        mIntakeExtension.set(ControlMode.PercentOutput, percentOutputTop);
        }

    @Override
    public void stop(){
        mIntakeGround.set(ControlMode.PercentOutput, 0.0);
        mIntakeExtension.set(ControlMode.PercentOutput, 0.0);
    }
    
    @Override
    public boolean checkSystem(){
        return false;  
    }
    @Override
    public void outputTelemetry(){
    }
}



