package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.ShooterAimingParameters;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Rotation2d;

import java.util.List;

/**
 * DriveUntilInRangeAction is an autonomous mode action that drives until the
 * shooter sees that a goal is in range. This is accomplished by looking for a
 * target with a supplied minimum distance and given a range of distances that
 * the robot will drive
 *
 * @see Superstructure
 * @see Action
 * @see Drive
 */
public class DriveUntilInRangeAction implements Action {

    private double mMinDistanceAway, mMaxDistanceToDrive, mVelocity;
    private double startingDistance;
    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();

    public DriveUntilInRangeAction(double velocity, double minDistanceAway, double maxDistanceToDrive) {
        mMaxDistanceToDrive = maxDistanceToDrive;
        mMinDistanceAway = minDistanceAway;
        mVelocity = velocity;
    }

    @Override
    public boolean isFinished() {
        if (getCurrentDistance() - startingDistance >= mMaxDistanceToDrive) {
            return true;
        }

        List<ShooterAimingParameters> params = mSuperstructure.getCachedAimingParams();
        System.out.println("#l: " + params.size());
        if (params.isEmpty()) {
            return false;
        }

        System.out.println("R: " + params.get(0).getRange());
        if (params.get(0).getRange() < mMinDistanceAway) {
            return true;
        }

        return false;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        System.out.println("Drive done, Setting drive to neutral");
        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void start() {
        startingDistance = getCurrentDistance();
        mDrive.setHighGear(false);
        mDrive.setVelocityHeadingSetpoint(mVelocity, Rotation2d.fromDegrees(0));
    }

    private double getCurrentDistance() {
        return (mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2;
    }
}
