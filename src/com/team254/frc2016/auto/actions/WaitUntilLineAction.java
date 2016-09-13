package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Drive;

/**
 * Sets the robot to stop at the next detected line
 */
public class WaitUntilLineAction implements Action {
    Drive mDrive = Drive.getInstance();
    int mCount = 0;

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        mDrive.setStopOnNextLine();
    }

}
