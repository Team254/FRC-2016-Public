package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Intake roller either intakes balls (towards the shooter assembly,
 * consisting of the Flywheel, Hood, and HoodRoller), or out-takes them to the
 * field. It is in one of these two states.
 * 
 * The ball is first picked up with the Intake then is fed to the Flywheel with
 * the HoodRoller. The Turret controls the direction that the ball is fired at.
 * Finally, the Hood controls the output angle and, conversely, trajectory.
 * 
 * @see Flywheel
 * @see Hood
 * @see HoodRoller
 * @see Turret
 * @see Superstructure
 */
public class Intake extends Subsystem {
    private CANTalon intake_talon_;
    private CANTalon fixed_talon_;
    private Solenoid deploy_solenoid_;
    private AnalogInput have_ball_sensor_;

    Intake() {
        intake_talon_ = new CANTalon(Constants.kIntakeTalonId);
        intake_talon_.enableBrakeMode(false);

        intake_talon_.setPID(Constants.kIntakeKp, Constants.kIntakeKi, Constants.kIntakeKd, Constants.kIntakeKf,
                Constants.kIntakeIZone, Constants.kIntakeRampRate, 0);
        intake_talon_.setProfile(0);
        intake_talon_.reverseSensor(false);
        intake_talon_.reverseOutput(false);
        intake_talon_.changeControlMode(TalonControlMode.Current);

        fixed_talon_ = new CANTalon(Constants.kFixedRollerTalonId);
        fixed_talon_.changeControlMode(TalonControlMode.PercentVbus);
        fixed_talon_.enableBrakeMode(false);
        deploy_solenoid_ = Constants.makeSolenoidForId(Constants.kIntakeSolenoidId);
        have_ball_sensor_ = new AnalogInput(Constants.kHaveBallSensorAnalogId);
    }

    /**
     * Positive intakes balls, negative exhausts
     * 
     * @param Set
     *            power for the deploy Talon
     * @param Set
     *            power for the roller Talon
     */
    public synchronized void setIntakeRoller(double outer_power, double fixed_power) {
        boolean has_ball = hasBall();
        if (has_ball) {
            intake_talon_.set(Math.max(-outer_power * Constants.kIntakeTargetCurrent, 0));
        } else {
            intake_talon_.set(-outer_power * Constants.kIntakeTargetCurrent);
        }
        fixed_talon_.set(-fixed_power);
    }

    public synchronized void setDeploy(boolean deploy) {
        deploy_solenoid_.set(deploy);
    }

    public synchronized boolean hasBall() {
        return have_ball_sensor_.getAverageVoltage() > 1.5;
    }

    @Override
    public synchronized void stop() {
        setIntakeRoller(0, 0);
        setDeploy(false);
    }

    @Override
    public void outputToSmartDashboard() {
        double voltage = have_ball_sensor_.getAverageVoltage();
        SmartDashboard.putBoolean("have_ball", hasBall());
        SmartDashboard.putNumber("have_ball_voltage", voltage);
        SmartDashboard.putNumber("intake_current", intake_talon_.getOutputCurrent());
    }

    @Override
    public void zeroSensors() {
        // no-op
    }
}
