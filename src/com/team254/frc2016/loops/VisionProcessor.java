package com.team254.frc2016.loops;

import com.team254.frc2016.RobotState;
import com.team254.frc2016.vision.VisionUpdate;
import com.team254.frc2016.vision.VisionUpdateReceiver;

/**
 * This function adds vision updates (from the Nexus smartphone) to a list in
 * RobotState. This helps keep track of goals detected by the vision system. The
 * code to determine the best goal to shoot at and prune old Goal tracks is in
 * GoalTracker.java
 * 
 * @see GoalTracker.java
 */
public class VisionProcessor implements Loop, VisionUpdateReceiver {
    static VisionProcessor instance_ = new VisionProcessor();
    VisionUpdate update_ = null;
    RobotState robot_state_ = RobotState.getInstance();

    public static VisionProcessor getInstance() {
        return instance_;
    }

    VisionProcessor() {
    }

    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
        VisionUpdate update;
        synchronized (this) {
            if (update_ == null) {
                return;
            }
            update = update_;
            update_ = null;
        }
        robot_state_.addVisionUpdate(update.getCapturedAtTimestamp(), update.getTargets());
    }

    @Override
    public void onStop() {
        // no-op
    }

    @Override
    public synchronized void gotUpdate(VisionUpdate update) {
        update_ = update;
    }

}
