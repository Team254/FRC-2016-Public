package com.team254.frc2016;

import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the
 * wheelbase as a differential drive (with a corrective factor to account for
 * the inherent skidding of the center 4 wheels quasi-kinematically).
 */

public class Kinematics {
    private static final double kEpsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit (less
     * accurate than below, but useful for predicting motion)
     */
    public static RigidTransform2d.Delta forwardKinematics(double left_wheel_delta, double right_wheel_delta) {
        double linear_velocity = (left_wheel_delta + right_wheel_delta) / 2;
        double delta_v = (right_wheel_delta - left_wheel_delta) / 2;
        double delta_rotation = delta_v * 2 * Constants.kTrackScrubFactor / Constants.kTrackEffectiveDiameter;
        return new RigidTransform2d.Delta(linear_velocity, 0, delta_rotation);
    }

    /**
     * Forward kinematics using encoders and explicitly measured rotation (ex.
     * from gyro)
     */
    public static RigidTransform2d.Delta forwardKinematics(double left_wheel_delta, double right_wheel_delta,
            double delta_rotation_rads) {
        return new RigidTransform2d.Delta((left_wheel_delta + right_wheel_delta) / 2, 0, delta_rotation_rads);
    }

    /** Append the result of forward kinematics to a previous pose. */
    public static RigidTransform2d integrateForwardKinematics(RigidTransform2d current_pose, double left_wheel_delta,
            double right_wheel_delta, Rotation2d current_heading) {
        RigidTransform2d.Delta with_gyro = forwardKinematics(left_wheel_delta, right_wheel_delta,
                current_pose.getRotation().inverse().rotateBy(current_heading).getRadians());
        return current_pose.transformBy(RigidTransform2d.fromVelocity(with_gyro));
    }

    public static class DriveVelocity {
        public final double left;
        public final double right;

        public DriveVelocity(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public static DriveVelocity inverseKinematics(RigidTransform2d.Delta velocity) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new DriveVelocity(velocity.dx, velocity.dx);
        }
        double delta_v = Constants.kTrackEffectiveDiameter * velocity.dtheta / (2 * Constants.kTrackScrubFactor);
        return new DriveVelocity(velocity.dx - delta_v, velocity.dx + delta_v);
    }
}
