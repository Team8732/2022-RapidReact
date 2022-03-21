package frc.team8732.robot;

import java.util.Optional;

import frc.team8732.lib.geometry.Rotation2d;
import frc.team8732.robot.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoModeSelector {

    enum StartingPosition {
        HANGER_SIDE, TERMINAL_SIDE
    }

    enum DesiredMode {
        DO_NOTHING, TEST_TRAJECTORY, THREE_BALL,FOUR_BALL,FIVE_BALL,STEAL_BALL,SHOOT_ONE
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private Rotation2d autoRotation = Rotation2d.fromDegrees(147);

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Terminal Side Box", StartingPosition.TERMINAL_SIDE);
        mStartPositionChooser.addOption("Hanger Side Box", StartingPosition.HANGER_SIDE);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Trajectory", DesiredMode.TEST_TRAJECTORY);
        mModeChooser.addOption("Four Ball Auto", DesiredMode.FOUR_BALL);
        mModeChooser.addOption("Five Ball Trajectory", DesiredMode.FIVE_BALL);
        mModeChooser.addOption("Steal Ball Trajectory", DesiredMode.STEAL_BALL);
        mModeChooser.addOption("Shoot One", DesiredMode.SHOOT_ONE);

        
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();

        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }

        if (startingPosition == null) {
            startingPosition = StartingPosition.TERMINAL_SIDE;
        }

        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private boolean startingTerminalSide(StartingPosition position) {
        return position == StartingPosition.TERMINAL_SIDE;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingAutoMode());
            case TEST_TRAJECTORY:
                return Optional.of(new TestTrajectoryFollowingMode());
            case THREE_BALL:
                if(startingTerminalSide(position)){
                    return Optional.of(new DoNothingAutoMode()); // Starting terminal side three ball auto
                }else{
                    return Optional.of(new DoNothingAutoMode()); // Starting hanger side three ball auto
                } 
            case FOUR_BALL:
                autoRotation = Rotation2d.fromDegrees(135);
                return Optional.of(new FourBallAutoMode());
            case FIVE_BALL:
                autoRotation = Rotation2d.fromDegrees(-147);
                 return Optional.of(new FiveBallAutoMode());
            case STEAL_BALL:
                autoRotation = Rotation2d.fromDegrees(135);
                return Optional.of(new TwoBallSteal());
            case SHOOT_ONE:
            return Optional.of(new ShootDoNothing());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public Rotation2d getAutoRotation(){
        return autoRotation;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
        SmartDashboard.putString("Auto Starting Rot", getAutoRotation().toString());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
}