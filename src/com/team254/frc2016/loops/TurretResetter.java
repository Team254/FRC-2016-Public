package com.team254.frc2016.loops;

import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.Turret;

/**
 * Periodically checks if the pivoting turret hits either extreme (remember, the
 * turret cannot spin in a complete circle). There are bumper switches at both
 * extremes of the turret, and this checks if the bumper switches are pressed.
 * If so, the turret re-centers itself.
 */
public class TurretResetter implements Loop {
    Superstructure mSuperstructure = Superstructure.getInstance();
    Turret mTurret = Superstructure.getInstance().getTurret();

    @Override
    public void onStart() {
        // no-op
    }

    @Override
    public void onLoop() {
        if (mTurret.getForwardLimitSwitch()) {
            mSuperstructure.resetTurretAtMax();
        } else if (mTurret.getReverseLimitSwitch()) {
            mSuperstructure.resetTurretAtMin();
        }
    }

    @Override
    public void onStop() {
        // no-op
    }

}
