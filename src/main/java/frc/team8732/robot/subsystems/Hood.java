
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import frc.team8732.lib.drivers.TalonSRXFactory;
import frc.team8732.lib.util.ReflectingCSVWriter;
import frc.team8732.lib.util.Util;
import frc.team8732.robot.Constants;

/* This is the Hood subsystem for the 2022 Robot. It contains information 
(variables, constructors, methods etc.) in order to allow the hood to function.*/

public class Hood extends Subsystem {
  private static Hood mInstance;

  public synchronized static Hood getInstance() {
    if(mInstance == null){
        mInstance = new Hood();
    }

    return mInstance;
  }

  private TalonSRX mHoodMotor;

  // Set the default
  public Hood() {
    mHoodMotor = TalonSRXFactory.createDefaultTalon(Constants.kHoodMotor);
  }

  // Control States
    private HoodControlState mHoodControlState = HoodControlState.OPEN_LOOP;
    private HoodControlMode mHoodControlMode = HoodControlMode.MANUAL;


  public enum HoodControlState {
    OPEN_LOOP,  // open loop voltage control 
    MOTIONMAGIC
  }

 public enum HoodControlMode { 
  MANUAL
}

// Hardware States
private final int kMotionMagicSlot = 0;

//"CHECK" SYSTEM (personal)
public static class PeriodicIO {
    // MOTOR OUTPUT
    public double demand = 0.0;

    // INPUTS+
    public double timestamp;
    public double output_voltage;
    public double supply_current;
    public double stator_current;
    public double position_ticks = 0.0; 
    public double position_degrees = Constants.kHoodPositionInDegrees; //TODO Ask Babob from the ground      
}

private PeriodicIO mPeriodicIO;
private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;


private final double kHoodOutputToEncoderRatio = 392.0/18.0; //ratio: 392:18
private final double kHoodRevolutionsToEncoderTicks = kHoodOutputToEncoderRatio * Constants.kHoodEncoderPPR;
private final double kHoodDegreesToEncoderTicks = kHoodRevolutionsToEncoderTicks / 360.0; //(personal - circle constant)
private final double hoodAllowableErrorDegrees = (int) kHoodOutputToEncoderRatio * .1;
private final double kMINHoodPositionInDegrees = Constants.kHoodPositionInDegrees;
private final double kMAXHoodPositionInDegrees = 55.00; //TODO (MAX ANGLE)


private double homePositionAngleDegrees = Constants.kHoodPositionInDegrees;  

// Subsytem Looper Methods
@Override
    public void readPeriodicInputs() {   
        mPeriodicIO.timestamp = Timer.getFPGATimestamp(); 

        mPeriodicIO.output_voltage = mHoodMotor.getMotorOutputVoltage();
        mPeriodicIO.supply_current = mHoodMotor.getSupplyCurrent();
        mPeriodicIO.stator_current = mHoodMotor.getStatorCurrent();

        mPeriodicIO.position_ticks = mHoodMotor.getSelectedSensorPosition(kMotionMagicSlot); //(personal) raw sensor units is position ticks 

        mPeriodicIO.position_degrees = positionTicksToDegrees(mPeriodicIO.position_ticks);

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

   
private double feedForward = .07;
    @Override
    public void writePeriodicOutputs() {
        if (mHoodControlState == HoodControlState.OPEN_LOOP) {
            mHoodMotor.set(ControlMode.PercentOutput, mPeriodicIO.demand);
        } else if (mHoodControlState == HoodControlState.MOTIONMAGIC) {
            mHoodMotor.set(ControlMode.MotionMagic, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, feedForward);
        }
    }

// Hood Conversion Methods

    /**
     * @param position_ticks
     * @return degrees
     */

    public double positionTicksToDegrees(double position_ticks) { //raw sensor units to degrees
      return position_ticks / kHoodDegreesToEncoderTicks + homePositionAngleDegrees;
  }

  /**
   * @param degrees
   * @return PositionTicks
   */
  public double degreesToPositionTicks(double position_degrees) {
      return position_degrees * kHoodDegreesToEncoderTicks; 
  }

// Hood Accesor Methods

  //current position ticks
  public synchronized double getPositionNativeUnits() { 
      return mPeriodicIO.position_ticks;
    }

    //current degrees
    public synchronized double getDegrees(){  
      return  mPeriodicIO.position_degrees;
    } 

    //desired degree
  public synchronized double getDegreesDemand() {  
      return positionTicksToDegrees(mPeriodicIO.demand);
  }

  //Hood Modifier Methods 
  public synchronized void setOpenLoop(double power) {
    if (mHoodControlState != HoodControlState.OPEN_LOOP) {
        mHoodControlState = HoodControlState.OPEN_LOOP;
    }

    mPeriodicIO.demand = power;
}

public synchronized void setPositionDegrees(double position_degrees) {
    if (mHoodControlMode != HoodControlMode.MANUAL) {
        mHoodControlMode = HoodControlMode.MANUAL;
        }

    if (position_degrees > kMAXHoodPositionInDegrees) {
        position_degrees = kMAXHoodPositionInDegrees;
    }

    if (position_degrees < kMINHoodPositionInDegrees) {
       position_degrees = kMINHoodPositionInDegrees;
    }

    mPeriodicIO.demand = degreesToPositionTicks(position_degrees);
}

public synchronized boolean isAtSetpoint() {
    return Util.epsilonEquals(getDegrees(), getDegreesDemand(),
            hoodAllowableErrorDegrees);
}

  @Override
  public void stop() {
    mHoodMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public boolean checkSystem() {
    return false;
  }


  @Override
  public void outputTelemetry() {
    
  }


}
