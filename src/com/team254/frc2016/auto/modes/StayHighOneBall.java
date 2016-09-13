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
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class StayHighOneBall extends AutoModeBase {
    Superstructure mSuperstructure = Superstructure.getInstance();

    ShooterAimingParameters mHint;
    private final boolean mShouldDriveBack;
    private final double kDistanceToDrive = 200;

    public static final double DISTANCE_TO_DROP_ARM = 100;

    public StayHighOneBall(ShooterAimingParameters hint, boolean shouldDriveBack) {
        mHint = hint;
        mShouldDriveBack = shouldDriveBack;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));
        mSuperstructure.setWantedState(WantedState.WANT_TO_KEEP_SPINNING);

        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 48.0));
        first_path.add(new Waypoint(new Translation2d(kDistanceToDrive, 0), 48.0));

        List<Waypoint> return_path = new ArrayList<>();
        return_path.add(new Waypoint(new Translation2d(kDistanceToDrive, 0), 48.0));
        return_path.add(new Waypoint(new Translation2d(18, 0), 48.0));

        runAction(new WaitAction(1.0));
        runAction(new FollowPathAction(new Path(first_path), false));

        runAction(new StartAutoAimingAction());
        runAction(new PointTurretAction(mHint));
        runAction(new ShootWhenReadyAction());

        if (mShouldDriveBack) {
            runAction(new FollowPathAction(new Path(return_path), true));
        }
    }
}
