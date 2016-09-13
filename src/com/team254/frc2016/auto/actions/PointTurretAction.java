package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.ShooterAimingParameters;
import com.team254.frc2016.subsystems.Superstructure;

/**
 * Action for aiming the turret at a specified target
 */
public class PointTurretAction implements Action {
    private ShooterAimingParameters mHint;
    private boolean mIsDone;
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    public PointTurretAction(ShooterAimingParameters hint) {
        mHint = hint;
        mIsDone = false;
    }

    @Override
    public boolean isFinished() {
        return (mIsDone && mSuperstructure.HasTarget());
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        mSuperstructure.clearTurretManualPositionSetpoint();
    }

    @Override
    public void start() {
        mSuperstructure.setTurretManualPositionSetpoint(mHint);
        mIsDone = true;
    }
}
