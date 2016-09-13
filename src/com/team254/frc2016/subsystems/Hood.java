package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.MA3Encoder;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.SynchronousPID;

import edu.wpi.first.wpilibj.ContinuousRotationServo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Hood assembly controls the launch angle of the ball. Additionally, it
 * must be lowered whenever the robot is crossing a defense. There are several
 * parameters accessible to the rest of the robot code: the hood angle, whether
 * or not the hood is stowed.
 * 
 * The ball is first picked up with the Intake then is fed to the Flywheel with
 * the HoodRoller. The Turret controls the direction that the ball is fired at.
 * Finally, the Hood controls the output angle and, conversely, trajectory.
 * 
 * This is a member of the Superstructure superclass.
 * 
 * @see Flywheel
 * @see Intake
 * @see HoodRoller
 * @see Turret
 * @see Superstructure
 */
public class Hood extends Subsystem {
    private ContinuousRotationServo left_servo_;
    private ContinuousRotationServo right_servo_;
    private ContinuousRotationServo test_servo_;
    private MA3Encoder encoder_;
    private Solenoid stow_solenoid_;
    private boolean has_homed_;
    private SynchronousPID pid_;

    enum ControlMode {
        HOMING, OPEN_LOOP, POSITION
    }

    ControlMode control_mode_;

    Loop hood_loop_ = new Loop() {
        static final double kHomingTimeSeconds = 1.0;
        ControlMode last_iteration_control_mode_ = ControlMode.OPEN_LOOP;
        double homing_start_time_ = 0;

        @Override
        public void onLoop() {
            synchronized (Hood.this) {
                if (control_mode_ == ControlMode.HOMING) {
                    if (control_mode_ != last_iteration_control_mode_) {
                        startHoming();
                        homing_start_time_ = Timer.getFPGATimestamp();
                    } else if (Timer.getFPGATimestamp() >= homing_start_time_ + kHomingTimeSeconds) {
                        stopHoming(true);
                    }
                } else if (control_mode_ == ControlMode.POSITION) {
                    set(pid_.calculate(getAngle().getDegrees()));
                }
                last_iteration_control_mode_ = control_mode_;
            }
        }

        @Override
        public void onStart() {
            synchronized (Hood.this) {
                if (!has_homed_) {
                    control_mode_ = ControlMode.HOMING;
                }
            }
        }

        @Override
        public void onStop() {
            synchronized (Hood.this) {
                if (control_mode_ == ControlMode.HOMING) {
                    stopHoming(false);
                }
            }
        }
    };

    Hood() {
        left_servo_ = new ContinuousRotationServo(Constants.kOppositeSideServoPWM);
        right_servo_ = new ContinuousRotationServo(Constants.kSensorSideServoPWM);
        test_servo_ = new ContinuousRotationServo(Constants.kTestServoPWM);
        test_servo_.set(0.0);
        encoder_ = new MA3Encoder(Constants.kHoodEncoderDIO);
        pid_ = new SynchronousPID(Constants.kHoodKp, Constants.kHoodKi, Constants.kHoodKd);
        pid_.setDeadband(Constants.kHoodDeadband);
        pid_.setInputRange(Constants.kMinHoodAngle, Constants.kMaxHoodAngle);
        stow_solenoid_ = Constants.makeSolenoidForId(Constants.kHoodStowSolenoidId);

        has_homed_ = false;
        pid_.setSetpoint(Constants.kMinHoodAngle);
        control_mode_ = ControlMode.OPEN_LOOP;
    }

    Loop getLoop() {
        return hood_loop_;
    }

    /**
     * Sets the angle of the hood to a specified angle.
     * 
     * @param A
     *            set angle
     */
    synchronized void setDesiredAngle(Rotation2d angle) {
        if (control_mode_ != ControlMode.HOMING && control_mode_ != ControlMode.POSITION) {
            control_mode_ = ControlMode.POSITION;
            pid_.reset();
        }
        pid_.setSetpoint(angle.getDegrees());
    }

    /**
     * Gets the current angle of the hood.
     * 
     * @return The hood's current angle.
     */
    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                encoder_.getContinuousAngleDegrees() * Constants.kHoodGearReduction + Constants.kMinHoodAngle);
    }

    private synchronized void set(double power) {
        left_servo_.set(-power);
        right_servo_.set(power);
    }

    synchronized void setOpenLoop(double power) {
        if (control_mode_ != ControlMode.HOMING) {
            set(power);
            control_mode_ = ControlMode.OPEN_LOOP;
        }
    }

    /**
     * Sets the hood state such that it begins retracting
     */
    public synchronized void homeSystem() {
        control_mode_ = ControlMode.HOMING;
    }

    /**
     * Makes the hood assembly begin to retract, or home.
     */
    synchronized void startHoming() {
        control_mode_ = ControlMode.HOMING;
        set(-1.0);
    }

    /**
     * Changes the control state of the Hood assembly if the hood is fully
     * retracted. If not, the Hood state is set to Open Loop.
     * 
     * @param If
     *            the hood has fully retracted.
     */
    synchronized void stopHoming(boolean success) {
        if (success) {
            has_homed_ = true;
            control_mode_ = ControlMode.POSITION;
            zeroSensors();
        } else {
            control_mode_ = ControlMode.OPEN_LOOP;
        }
        set(0);
    }

    public synchronized boolean hasHomed() {
        return has_homed_;
    }

    public synchronized double getSetpoint() {
        return pid_.getSetpoint();
    }

    /**
     * @return If the hood position is within a set tolerance to a specified
     *         value.
     */
    public synchronized boolean isOnTarget() {
        return (has_homed_ && control_mode_ == ControlMode.POSITION
                && Math.abs(pid_.getError()) < Constants.kHoodOnTargetTolerance);
    }

    /**
     * To pass an obstacle, the robot's hood must be stowed and the state must
     * be Position. This function checks that the robot is safe to pass through
     * an obstacle.
     * 
     * @return If the robot is safe to pass through an obstacle
     */
    public synchronized boolean isSafe() {
        return (control_mode_ == ControlMode.POSITION && getAngle().getDegrees() < Constants.kHoodMaxSafeAngle
                && pid_.getSetpoint() < Constants.kHoodMaxSafeAngle);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("has_hood_homed", has_homed_);
        SmartDashboard.putNumber("hood_angle", getAngle().getDegrees());
        SmartDashboard.putNumber("hood_setpoint", pid_.getSetpoint());
        SmartDashboard.putBoolean("hood_on_target", isOnTarget());
        SmartDashboard.putNumber("hood_error", pid_.getSetpoint() - getAngle().getDegrees());
    }

    @Override
    public synchronized void stop() {
        pid_.reset();
        control_mode_ = ControlMode.OPEN_LOOP;
        set(0);
    }

    @Override
    public synchronized void zeroSensors() {
        encoder_.zero();
    }

    public boolean isStowed() {
        return stow_solenoid_.get();
    }

    void setStowed(boolean stow) {
        stow_solenoid_.set(stow);
    }

    public void setTestServoSpeed(double speed) {
        test_servo_.set(speed);
    }
}
