package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Drive;
import com.team254.lib.util.Path;

/**
 * Action for following a path defined by a Path object.
 * 
 * @see Drive
 * @see Path
 */
public class FollowPathAction implements Action {

    private Drive mDrive = Drive.getInstance();

    private Path mPath;
    private boolean mReversed;
    private boolean mHasStarted;

    public FollowPathAction(Path path, boolean reversed) {
        mPath = path;
        mReversed = reversed;
        mHasStarted = false;
    }

    @Override
    public boolean isFinished() {
        boolean done = mDrive.isFinishedPath() && mHasStarted;
        if (done) {
            System.out.println("Finished path");
        }
        return done;
    }

    @Override
    public void update() {
        mHasStarted = true;
    }

    @Override
    public void done() {
        mDrive.stop();
    }

    @Override
    public void start() {
        mDrive.followPath(mPath, mReversed);
    }

}
