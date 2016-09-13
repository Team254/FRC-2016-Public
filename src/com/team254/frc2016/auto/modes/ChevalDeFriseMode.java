package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.ShooterAimingParameters;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.Superstructure.WantedState;
import com.team254.frc2016.subsystems.UtilityArm;
import com.team254.lib.util.Path;
import com.team254.lib.util.Translation2d;
import com.team254.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.List;

/**
 * In this autonomous routine, the robot crosses the cheval de frise. There are
 * switchable modes in this routine- the driver can specify whether the robot
 * should return, if it should return on the right side low bar, and if it
 * should launch a single ball.
 */
public class ChevalDeFriseMode extends AutoModeBase {
    Superstructure mSuperstructure = Superstructure.getInstance();

    ShooterAimingParameters mHint;
    private final double kDistanceToCDF = 41.0;
    private final double kDistanceToDrive = 200;
    private boolean mShouldDriveBack;
    private boolean mComeBackRight;

    public ChevalDeFriseMode(ShooterAimingParameters hint, boolean shouldComeBack, boolean comeBackRight) {
        mHint = hint;
        mShouldDriveBack = shouldComeBack;
        mComeBackRight = comeBackRight;
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 36.0));
        first_path.add(new Waypoint(new Translation2d(kDistanceToCDF, 0), 36.0));

        List<Waypoint> second_path = new ArrayList<>();
        second_path.add(new Waypoint(new Translation2d(kDistanceToCDF, 0), 60.0));
        second_path.add(new Waypoint(new Translation2d(kDistanceToCDF + 60, 0), 120.0));
        second_path.add(new Waypoint(new Translation2d(kDistanceToDrive, 0), 120.0));

        double y_distance = (mComeBackRight ? -47 : 58.0);
        List<Waypoint> return_path = new ArrayList<>();
        return_path.add(new Waypoint(new Translation2d(kDistanceToDrive, 0), 120.0));
        return_path.add(new Waypoint(new Translation2d(kDistanceToDrive, y_distance), 60.0));
        return_path.add(new Waypoint(new Translation2d(160, y_distance), 60.0));
        return_path.add(new Waypoint(new Translation2d(0, y_distance), 60.0));

        runAction(new FollowPathAction(new Path(first_path), false));
        runAction(new GetLowAction());
        mSuperstructure.setWantedState(WantedState.WANT_TO_KEEP_SPINNING);
        runAction(new FollowPathAction(new Path(second_path), false));
        runAction(new StartAutoAimingAction());
        runAction(new PointTurretAction(mHint));
        runAction(new ShootWhenReadyAction());
        runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));

        if (mShouldDriveBack) {
            runAction(new FollowPathAction(new Path(return_path), true));
        }
    }
}
