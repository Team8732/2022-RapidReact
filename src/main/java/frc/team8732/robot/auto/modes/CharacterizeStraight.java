package frc.team8732.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import frc.team8732.lib.physics.DriveCharacterization;
import frc.team8732.robot.auto.AutoModeEndedException;
import frc.team8732.robot.auto.actions.CollectAccelerationData;
import frc.team8732.robot.auto.actions.CollectVelocityData;
import frc.team8732.robot.auto.actions.WaitAction;

public class CharacterizeStraight extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        runAction(new CollectVelocityData(velocityData,false, false));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationData(accelerationData,false, false));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }
}
