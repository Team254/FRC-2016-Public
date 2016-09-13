package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Superstructure;

/**
 * Shoots the ball when ready by setting the robot state machine's desired
 * state.
 * 
 * @return If the firing was successful
 */
public class ShootWhenReadyAction implements Action {

    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private int mNumShotsFiredAtStart;

    @Override
    public boolean isFinished() {
        return mSuperstructure.getNumShotsFired() > mNumShotsFiredAtStart;
    }

    @Override
    public void start() {
        mNumShotsFiredAtStart = mSuperstructure.getNumShotsFired();
        mSuperstructure.setWantsToFireWhenReady();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_DEPLOY);
    }
}
