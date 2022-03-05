// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team8732.robot.subsystems;

import frc.team8732.robot.Limelight;

/** This is the Superstructrue subsystem. It takes instances from all other subsystem to 
 * create action states that the robot will take throughout the match and centralize them in one space */
public class Superstructure extends Subsystem {
    private static Superstructure mInstance;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Shooter mShooter = Shooter.getInstance();
    private Limelight mLimelight = Limelight.getInstance();

    private Superstructure(){
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}
}
