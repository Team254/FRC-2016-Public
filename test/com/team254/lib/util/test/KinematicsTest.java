package com.team254.lib.util.test;

import static org.junit.Assert.*;

import org.junit.Test;

import com.team254.frc2016.Kinematics;
import com.team254.lib.util.RigidTransform2d;

public class KinematicsTest {
    private static final double kEps = 1E-9;

    @Test
    public void test() {
        double left_diff = 10.0;
        double right_diff = 13.0;
        RigidTransform2d.Delta movement = Kinematics.forwardKinematics(left_diff, right_diff);
        Kinematics.DriveVelocity velocity = Kinematics.inverseKinematics(movement);
        assertEquals(velocity.left, left_diff, kEps);
        assertEquals(velocity.right, right_diff, kEps);

        left_diff = 10.0;
        right_diff = 10.0;
        movement = Kinematics.forwardKinematics(left_diff, right_diff);
        velocity = Kinematics.inverseKinematics(movement);
        assertEquals(velocity.left, left_diff, kEps);
        assertEquals(velocity.right, right_diff, kEps);

        left_diff = 0.0;
        right_diff = 0.0;
        movement = Kinematics.forwardKinematics(left_diff, right_diff);
        velocity = Kinematics.inverseKinematics(movement);
        assertEquals(velocity.left, left_diff, kEps);
        assertEquals(velocity.right, right_diff, kEps);

        left_diff = -10.0;
        right_diff = 10.0;
        movement = Kinematics.forwardKinematics(left_diff, right_diff);
        velocity = Kinematics.inverseKinematics(movement);
        assertEquals(velocity.left, left_diff, kEps);
        assertEquals(velocity.right, right_diff, kEps);
    }

}
