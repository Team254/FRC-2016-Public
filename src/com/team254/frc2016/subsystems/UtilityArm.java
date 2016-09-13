package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.frc2016.loops.Loop;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls the CDF, Portcullis, and Hanging mechanisms
 */
public class UtilityArm extends Subsystem {

    private static UtilityArm sInstance = new UtilityArm();

    public static UtilityArm getInstance() {
        return sInstance;
    }

    public enum WantedState {
        /** Keep in the sizing box, only valid from the start of the match */
        STAY_IN_SIZE_BOX, LOW_BAR, // Arm down
        DRIVING, // Arm at mid position

        /**
         * Want to get the hooks up for hanging, but not actually hang quite yet
         */
        PREPARE_FOR_HANG,

        /** Want to pull up to hang now */
        PULL_UP_HANG,

        /** Want to get arm out of the way to challenge on the batter */
        BATTER_CHALLENGE,

    }

    private enum SystemState {
        /** Start of match state */
        SIZE_BOX,

        /** Dropping the arm past the adj hardstops */
        SIZE_BOX_TO_LOW_BAR,

        /** Lower intake to prevent porcullis mode from damaging it */
        DRIVE_TO_LOW_BAR,

        /** Dragging on the floor, past the adj hardstops */
        LOW_BAR,

        /** Pushing arm up against adj hardstops */
        DRIVE,

        /** Lift arm past adj hardstops, clearing bumper */
        LIFTING_ARM_FOR_HANG,

        /** Extend hooks to hanging bar */
        DEPLOY_HOOKS,

        /** Activate motors to pull robot up */
        HANG,

        LOW_BAR_WAIT_FOR_HARDSTOP_CLEARANCE,

        BATTER_CHALLENGE,
    }

    /**
     * If there's a bug, revert to low bar mode, it's the least likely to break
     * the robot
     */
    private static final SystemState PANIC_SYSTEM_STATE = SystemState.LOW_BAR;
    private static final WantedState PANIC_WANTED_STATE = WantedState.LOW_BAR;

    // Transitioning to low bar mode is the least likely to destroy the
    // robot.
    private WantedState mWantedState = PANIC_WANTED_STATE;
    private boolean mIsAllowedToHang = false;
    private boolean mIsSafeToDriveThroughPortcullis = false;

    private Solenoid mArmLiftSolenoid = Constants.makeSolenoidForId(Constants.kArmLiftSolenoidId);
    private Solenoid mAdjustableHardStopSolenoid = Constants.makeSolenoidForId(Constants.kAdjustableHardStopSolenoidId);
    private Solenoid mHookReleaseSolenoid = Constants.makeSolenoidForId(Constants.kHookReleaseSolenoidId);

    private CANTalon mMasterTalon = new CANTalon(Constants.kHangerMasterTalonId);
    private CANTalon mSlaveTalon = new CANTalon(Constants.kHangerSlaveTalonId);

    Loop mLoop = new Loop() {

        private SystemState mSystemState = PANIC_SYSTEM_STATE;

        /**
         * Every time we transition states, we update the current state start
         * time and the state changed boolean (for one cycle)
         */

        private double mCurrentStateStartTime;

        @Override
        public void onStart() {
            /** Leave the wanted state as it was set before enabling */
            mSystemState = stateForOnStart();
            mCurrentStateStartTime = Timer.getFPGATimestamp();
            mIsAllowedToHang = false;
            mIsSafeToDriveThroughPortcullis = false;
        }

        @Override
        public void onLoop() {
            double now = Timer.getFPGATimestamp();
            SystemState newState = getNextState(now);
            if (newState != mSystemState) {
                System.out.println("Utility Arm state " + mSystemState + " to " + newState);
                mSystemState = newState;
                mCurrentStateStartTime = now;
            }
            setOutputsForState(mSystemState);
        }

        @Override
        public void onStop() {
        }

        private SystemState getNextState(double now) {
            double timeSinceStateStart = now - mCurrentStateStartTime;
            switch (mSystemState) {
            case SIZE_BOX:
                return handleSizeBox();
            case SIZE_BOX_TO_LOW_BAR:
                return handleSizeBoxToLowBar(timeSinceStateStart);
            case DRIVE_TO_LOW_BAR:
                return handleDrivingToLowBar(timeSinceStateStart);
            case LOW_BAR:
                return handleLowBar();
            case DRIVE:
                return handleDriving();
            case LIFTING_ARM_FOR_HANG:
                return handleLiftingArmForHang(timeSinceStateStart);
            case DEPLOY_HOOKS:
                return handleDeployHooks();
            case HANG:
                return handleHang();
            case LOW_BAR_WAIT_FOR_HARDSTOP_CLEARANCE:
                return handleWaitForHardstopClearance(timeSinceStateStart);
            case BATTER_CHALLENGE:
                return handleBatterChallenge();
            default:
                System.out.println("Utility Arm unknown system state" + mSystemState);
                return PANIC_SYSTEM_STATE;
            }
        }
    };

    UtilityArm() {
        mMasterTalon.changeControlMode(TalonControlMode.Voltage);
        mMasterTalon.enableBrakeMode(true);
        mSlaveTalon.enableBrakeMode(true);
        mSlaveTalon.changeControlMode(TalonControlMode.Follower);
        mSlaveTalon.set(Constants.kHangerMasterTalonId);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("hanger_motor_master_current", mMasterTalon.getOutputCurrent());
        SmartDashboard.putNumber("hanger_motor_slave_current", mSlaveTalon.getOutputCurrent());
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

    public Loop getLoop() {
        return mLoop;
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    public synchronized boolean isAllowedToHang() {
        return mIsAllowedToHang;
    }

    public synchronized boolean isSafeToDriveThroughPortcullis() {
        return mIsSafeToDriveThroughPortcullis;
    }

    /**
     * Picks an appropriate safe initial state for the system after the robot
     * has been re-enabled.
     * 
     * @return What the state should be as the looper starts.
     */
    private synchronized SystemState stateForOnStart() {
        switch (mWantedState) {
        case STAY_IN_SIZE_BOX:
            return SystemState.SIZE_BOX;
        case LOW_BAR:
            return SystemState.LOW_BAR;
        case DRIVING:
            return SystemState.DRIVE;
        case PREPARE_FOR_HANG:
            return SystemState.LIFTING_ARM_FOR_HANG;
        case PULL_UP_HANG:
            return SystemState.HANG;
        case BATTER_CHALLENGE:
            return SystemState.BATTER_CHALLENGE;
        default:
            System.out.println("UtilityArm Unknown wanted state: " + mWantedState);
            mWantedState = PANIC_WANTED_STATE;
            return PANIC_SYSTEM_STATE;
        }
    }

    private synchronized SystemState handleSizeBox() {
        return mWantedState == WantedState.STAY_IN_SIZE_BOX ? SystemState.SIZE_BOX : SystemState.SIZE_BOX_TO_LOW_BAR;
    }

    private synchronized SystemState handleSizeBoxToLowBar(double timeSinceStateStart) {
        if (mWantedState == WantedState.STAY_IN_SIZE_BOX) {
            logIllegalWantedState(SystemState.SIZE_BOX_TO_LOW_BAR, mWantedState);
        }
        return timeSinceStateStart >= Constants.kUtilityArmDropTime ? SystemState.LOW_BAR
                : SystemState.SIZE_BOX_TO_LOW_BAR;
    }

    private synchronized SystemState handleDrivingToLowBar(double timeSinceStateStart) {
        switch (mWantedState) {
        case PREPARE_FOR_HANG:
        case BATTER_CHALLENGE:
        case LOW_BAR:
            return timeSinceStateStart >= Constants.kUtilityArmDropTime ? SystemState.LOW_BAR
                    : SystemState.DRIVE_TO_LOW_BAR;
        case DRIVING:
            return SystemState.DRIVE;
        case STAY_IN_SIZE_BOX:
        case PULL_UP_HANG:
        default:
            logIllegalWantedState(SystemState.DRIVE_TO_LOW_BAR, mWantedState);
            return SystemState.DRIVE_TO_LOW_BAR;
        }
    }

    private synchronized SystemState handleLowBar() {
        mIsSafeToDriveThroughPortcullis = true;
        switch (mWantedState) {
        case LOW_BAR:
            return SystemState.LOW_BAR;
        case DRIVING:
            mIsSafeToDriveThroughPortcullis = false;
            return SystemState.DRIVE;
        case PREPARE_FOR_HANG:
            mIsSafeToDriveThroughPortcullis = false;
            return SystemState.LIFTING_ARM_FOR_HANG;
        case BATTER_CHALLENGE:
            mIsSafeToDriveThroughPortcullis = false;
            return SystemState.LOW_BAR_WAIT_FOR_HARDSTOP_CLEARANCE;
        case STAY_IN_SIZE_BOX:
        case PULL_UP_HANG:
        default:
            logIllegalWantedState(SystemState.LOW_BAR, mWantedState);
            return SystemState.LOW_BAR;
        }
    }

    private synchronized SystemState handleDriving() {
        switch (mWantedState) {
        case LOW_BAR:
        case BATTER_CHALLENGE:
        case PREPARE_FOR_HANG:
            return SystemState.DRIVE_TO_LOW_BAR;
        case DRIVING:
            return SystemState.DRIVE;
        case STAY_IN_SIZE_BOX:
        case PULL_UP_HANG:
        default:
            logIllegalWantedState(SystemState.DRIVE, mWantedState);
            return SystemState.DRIVE;
        }
    }

    private synchronized SystemState handleLiftingArmForHang(double timeSinceStateStart) {
        if (mWantedState != WantedState.PREPARE_FOR_HANG) {
            logIllegalWantedState(SystemState.LIFTING_ARM_FOR_HANG, mWantedState);
        }
        return timeSinceStateStart >= Constants.kUtilityArmRaiseTime ? SystemState.DEPLOY_HOOKS
                : SystemState.LIFTING_ARM_FOR_HANG;
    }

    private synchronized SystemState handleDeployHooks() {
        mIsAllowedToHang = true;
        if (mWantedState == WantedState.PREPARE_FOR_HANG) {
            return SystemState.DEPLOY_HOOKS;
        } else if (mWantedState == WantedState.PULL_UP_HANG) {
            return SystemState.HANG;
        } else {
            logIllegalWantedState(SystemState.DEPLOY_HOOKS, mWantedState);
            return SystemState.DEPLOY_HOOKS;
        }
    }

    private synchronized SystemState handleHang() {
        mIsAllowedToHang = true;
        if (mWantedState == WantedState.PREPARE_FOR_HANG) {
            return SystemState.DEPLOY_HOOKS;
        } else if (mWantedState == WantedState.PULL_UP_HANG) {
            return SystemState.HANG;
        } else {
            logIllegalWantedState(SystemState.HANG, mWantedState);
            return SystemState.DEPLOY_HOOKS;
        }
    }

    private synchronized SystemState handleBatterChallenge() {
        switch (mWantedState) {
        case LOW_BAR:
        case DRIVING:
            return SystemState.DRIVE_TO_LOW_BAR;
        case PREPARE_FOR_HANG:
            return SystemState.LIFTING_ARM_FOR_HANG;
        case BATTER_CHALLENGE:
            return SystemState.BATTER_CHALLENGE;
        case PULL_UP_HANG:
        case STAY_IN_SIZE_BOX:
        default:
            logIllegalWantedState(SystemState.BATTER_CHALLENGE, mWantedState);
            return SystemState.BATTER_CHALLENGE;
        }
    }

    private synchronized SystemState handleWaitForHardstopClearance(double timeSinceStateStart) {
        switch (mWantedState) {
        case LOW_BAR:
        case DRIVING:
            return SystemState.DRIVE_TO_LOW_BAR;
        case PREPARE_FOR_HANG:
        case PULL_UP_HANG:
            return timeSinceStateStart >= Constants.kUtilityArmHardStopsMoveForRaiseArmDelay ? SystemState.DEPLOY_HOOKS
                    : SystemState.LOW_BAR_WAIT_FOR_HARDSTOP_CLEARANCE;
        case BATTER_CHALLENGE:
            return timeSinceStateStart >= Constants.kUtilityArmHardStopsMoveForRaiseArmDelay
                    ? SystemState.BATTER_CHALLENGE : SystemState.LOW_BAR_WAIT_FOR_HARDSTOP_CLEARANCE;
        case STAY_IN_SIZE_BOX:
        default:
            logIllegalWantedState(SystemState.BATTER_CHALLENGE, mWantedState);
            return SystemState.LOW_BAR_WAIT_FOR_HARDSTOP_CLEARANCE;
        }
    }

    private void logIllegalWantedState(SystemState systemState, WantedState wantedState) {
        System.out.println("Illegal Wanted state from " + systemState + " to " + wantedState);
    }

    private void setOutputsForState(SystemState systemState) {
        switch (systemState) {
        case SIZE_BOX_TO_LOW_BAR:
        case DRIVE_TO_LOW_BAR:
        case LOW_BAR_WAIT_FOR_HARDSTOP_CLEARANCE:
        case LOW_BAR:
            setOutputs(ArmOutput.ARM_DOWN, AdjustableHardstopOutput.PREVENT_HANG, HookReleaseOutput.HOOKS_HELD_IN);
            mMasterTalon.set(0.0);
            break;
        case SIZE_BOX:
        case DRIVE:
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.PREVENT_HANG, HookReleaseOutput.HOOKS_HELD_IN);
            mMasterTalon.set(0.0);
            break;
        case BATTER_CHALLENGE:
        case LIFTING_ARM_FOR_HANG:
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.ALLOW_HANG, HookReleaseOutput.HOOKS_HELD_IN);
            mMasterTalon.set(0.0);
            break;
        case DEPLOY_HOOKS:
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.ALLOW_HANG, HookReleaseOutput.HOOKS_RELEASED);
            mMasterTalon.set(0.0);
            break;
        case HANG:
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.ALLOW_HANG, HookReleaseOutput.HOOKS_RELEASED);
            mMasterTalon.set(-12.0);
            break;
        default:
            mMasterTalon.set(0.0);
            System.out.println("Utility arm unknown state for output: " + systemState);
        }
    }

    /**
     * Always set all the outputs so we aren't tempted to merge states of the
     * solenoid outputs (which lose state on disable) and this state machine
     * (which keeps state through disable).
     */
    private void setOutputs(ArmOutput armOutput, AdjustableHardstopOutput adjustableHardstopOutput,
            HookReleaseOutput hookReleaseOutput) {
        mArmLiftSolenoid.set(armOutput.value);
        mAdjustableHardStopSolenoid.set(adjustableHardstopOutput.value);
        mHookReleaseSolenoid.set(hookReleaseOutput.value);
    }

    /**
     * These enums strongly type solenoid outputs to their respective solenoid
     * directions
     **/
    private enum ArmOutput {
        ARM_UP(false), ARM_DOWN(!ARM_UP.value);

        final boolean value;

        ArmOutput(boolean value) {
            this.value = value;
        }
    }

    private enum AdjustableHardstopOutput {
        PREVENT_HANG(false), ALLOW_HANG(!PREVENT_HANG.value);

        final boolean value;

        AdjustableHardstopOutput(boolean value) {
            this.value = value;
        }
    }

    private enum HookReleaseOutput {
        HOOKS_HELD_IN(false), HOOKS_RELEASED(!HOOKS_HELD_IN.value);

        final boolean value;

        HookReleaseOutput(boolean value) {
            this.value = value;
        }
    }
}
