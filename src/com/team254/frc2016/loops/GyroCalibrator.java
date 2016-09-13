package com.team254.frc2016.loops;

import com.team254.frc2016.subsystems.Drive;
import com.team254.lib.util.ADXRS453_Gyro;

import edu.wpi.first.wpilibj.Timer;

/**
 * Routine that recalibrates the gyroscope every 5 seconds. The gyroscope helps
 * the robot drive in a straight line despite obstacles and compensates for
 * bumps and obstacles.
 */
public class GyroCalibrator implements Loop {
    ADXRS453_Gyro mGyro = Drive.getInstance().getGyro();

    double mCalibrationStartTime = 0;

    @Override
    public void onStart() {
        // no-op
    }

    @Override
    public void onLoop() {
        double now = Timer.getFPGATimestamp();
        // Keep re-calibrating the gyro every 5 seconds
        if (now - mCalibrationStartTime > ADXRS453_Gyro.kCalibrationSampleTime) {
            mGyro.endCalibrate();
            System.out.println("Gyro calibrated, new zero is " + mGyro.getCenter());
            mCalibrationStartTime = now;
            mGyro.startCalibrate();
        }
    }

    @Override
    public void onStop() {
        mGyro.cancelCalibrate();
    }

}
