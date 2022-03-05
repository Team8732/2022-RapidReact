// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.team8732.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team8732.lib.drivers.TalonSRXFactory;
import frc.team8732.robot.Constants;
 
/** Add your docs here. */
 
public class Intake extends Subsystem {
    public static final double percentOutput = 0;
    private static Intake mInstance;
 
//get Instances
public synchronized static Intake getInstance() {
    if (mInstance == null){
        mInstance = new Intake();
    }
      return mInstance;
}
 
//Hardware(motor controllers/Talons)
   private final TalonSRX mSpicyIntake;
   public double setIntakeSpeedPercent(double percentOutput) {
    return percentOutput;
}
   public Intake(){
    mSpicyIntake = TalonSRXFactory.createDefaultTalon(Constants.kSpicyIntake);
   }
 
 
@Override
public void stop(){
    mSpicyIntake.set(ControlMode.PercentOutput, 0.0);
 
    //TODO Auto-generated method stub
}
 
@Override
public boolean checkSystem(){
    return false;  
    //TODO Auto-generated method stub
}
@Override
public void outputTelemetry(){
    ////TODO Auto-generated method stub
}
 
}

