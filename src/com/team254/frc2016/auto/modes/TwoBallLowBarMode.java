package com.team254.frc2016.auto.modes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.ShooterAimingParameters;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.UtilityArm;
import com.team254.frc2016.subsystems.Superstructure.WantedState;
import com.team254.lib.util.Path;
import com.team254.lib.util.Path.Waypoint;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

/**
 * This routine goes under the low bar and attempts to fire two balls.
 */
public class TwoBallLowBarMode extends AutoModeBase {
    Superstructure mSuperstructure = Superstructure.getInstance();
    Drive mDrive = Drive.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {
        ShooterAimingParameters hint = new ShooterAimingParameters(150, Rotation2d.fromDegrees(-55), -1);

        System.out.println("Starting 2 ball auto mode...");
        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 120.0));
        first_path.add(new Waypoint(new Translation2d(24, 0), 120.0));
        first_path.add(new Waypoint(new Translation2d(24, 18), 60.0));
        first_path.add(new Waypoint(new Translation2d(100, 18), 120.0, "PopHood"));
        first_path.add(new Waypoint(new Translation2d(215, 18), 120.0));

        List<Waypoint> second_path = new ArrayList<>();
        second_path.add(new Waypoint(new Translation2d(215, 18), 120.0));
        second_path.add(new Waypoint(new Translation2d(150, 18), 60.0));
        second_path.add(new Waypoint(new Translation2d(70, 18), 84.0));
        second_path.add(new Waypoint(new Translation2d(56, 18), 84.0));
        second_path.add(new Waypoint(new Translation2d(56, 24), 84.0));
        second_path.add(new Waypoint(new Translation2d(18, 26), 84.0));

        List<Waypoint> third_path = new ArrayList<>();
        third_path.add(new Waypoint(new Translation2d(18, 26), 120.0));
        third_path.add(new Waypoint(new Translation2d(24, 18), 120.0));
        third_path.add(new Waypoint(new Translation2d(100, 18), 120.0, "PopHood"));
        third_path.add(new Waypoint(new Translation2d(205, 18), 120.0));

        mSuperstructure.setWantedState(WantedState.WANT_TO_STOW);
        mSuperstructure.setWantsToRunIntake();
        mSuperstructure.deployIntake();
        runAction(new ParallelAction(Arrays.asList(new GetLowAction(), new WaitAction(0.75))));

        runAction(new ParallelAction(Arrays.asList(new FollowPathAction(new Path(first_path), false),
                new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PopHood"), new StartAutoAimingAction(),
                        new PointTurretAction(hint))))));
        mSuperstructure.setWantsToStopIntake();
        runAction(new ShootWhenReadyAction());
        mSuperstructure.setWantedState(WantedState.WANT_TO_STOW);
        mSuperstructure.setWantsToRunIntake();
        runAction(new FollowPathAction(new Path(second_path), true));
        runAction(new ParallelAction(Arrays.asList(new FollowPathAction(new Path(third_path), false),
                new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PopHood"), new StartAutoAimingAction(),
                        new PointTurretAction(hint))))));
        runAction(new ShootWhenReadyAction());
        mSuperstructure.setWantedState(WantedState.WANT_TO_DEPLOY);
        mSuperstructure.setWantsToStopIntake();
        runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));
    }
}
