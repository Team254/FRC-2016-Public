package com.team254.frc2016.loops;

import com.team254.frc2016.Kinematics;
import com.team254.frc2016.RobotState;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.Turret;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope orientation, and velocity, among
 * various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    RobotState robot_state_ = RobotState.getInstance();
    Drive drive_ = Drive.getInstance();
    Turret turret_ = Superstructure.getInstance().getTurret();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;

    @Override
    public void onStart() {
        left_encoder_prev_distance_ = drive_.getLeftDistanceInches();
        right_encoder_prev_distance_ = drive_.getRightDistanceInches();
    }

    @Override
    public void onLoop() {
        double time = Timer.getFPGATimestamp();
        double left_distance = drive_.getLeftDistanceInches();
        double right_distance = drive_.getRightDistanceInches();
        Rotation2d gyro_angle = drive_.getGyroAngle();
        Rotation2d turret_angle = turret_.getAngle();
        RigidTransform2d odometry = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        RigidTransform2d.Delta velocity = Kinematics.forwardKinematics(drive_.getLeftVelocityInchesPerSec(),
                drive_.getRightVelocityInchesPerSec());
        robot_state_.addObservations(time, odometry, turret_angle, velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }

    @Override
    public void onStop() {
        // no-op
    }

}
