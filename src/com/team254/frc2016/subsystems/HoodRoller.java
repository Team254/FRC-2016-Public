package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.frc2016.loops.Loop;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is the hood roller, which feeds balls from the intake to the shooter.
 * More precisely, the ball is fed to the Flywheel, and the Hood controls the
 * output angle and, conversely, trajectory. The HoodRoller can also eject balls
 * by reversing the direction of the rollers.
 * 
 * The ball is first picked up with the Intake then is fed to the Flywheel with
 * the HoodRoller. The Turret controls the direction that the ball is fired at.
 * Finally, the Hood controls the output angle and, conversely, trajectory.
 * 
 * This is a member of the Superstructure superclass.
 * 
 * @see Flywheel
 * @see Intake
 * @see Hood
 * @see Turret
 * @see Superstructure
 */
public class HoodRoller extends Subsystem {
    CANTalon mRollerTalon;
    AnalogInput mBallReadySensor;

    protected static enum WantedState {
        WANTS_STOP, WANTS_INTAKE, WANTS_REVERSE, WANTS_SHOOT
    };

    WantedState mWantedState = WantedState.WANTS_STOP;

    Loop mLoop = new Loop() {

        @Override
        public void onStart() {
            mWantedState = WantedState.WANTS_STOP;
        }

        @Override
        public void onLoop() {
            synchronized (HoodRoller.this) {
                switch (mWantedState) {
                case WANTS_INTAKE:
                    if (isBallPresent()) {
                        stop();
                    } else {
                        mRollerTalon.set(10.5);
                    }
                    break;
                case WANTS_REVERSE:
                    mRollerTalon.set(-12.0);
                    break;
                case WANTS_SHOOT:
                    mRollerTalon.set(10.5);
                    break;
                case WANTS_STOP:
                default:
                    stop();
                    break;
                }
            }
        }

        @Override
        public void onStop() {
        }

    };

    HoodRoller() {
        mRollerTalon = new CANTalon(Constants.kHoodRollerTalonId);
        mRollerTalon.enableBrakeMode(true);
        mRollerTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mBallReadySensor = new AnalogInput(Constants.kBallReadyAnalogId);
    }

    public Loop getLoop() {
        return mLoop;
    }

    public synchronized void intake() {
        mWantedState = WantedState.WANTS_INTAKE;
    }

    public synchronized void reverse() {
        mWantedState = WantedState.WANTS_REVERSE;
    }

    public synchronized void shoot() {
        mWantedState = WantedState.WANTS_SHOOT;
    }

    public boolean isBallPresent() {
        return mBallReadySensor.getAverageVoltage() > 2.0;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("ball_ready_sensor_voltage", mBallReadySensor.getAverageVoltage());
        SmartDashboard.putBoolean("ball_ready_sensor", isBallPresent());
    }

    @Override
    public synchronized void stop() {
        mRollerTalon.set(0.0);
        mWantedState = WantedState.WANTS_STOP;
    }

    @Override
    public void zeroSensors() {
    }

}
