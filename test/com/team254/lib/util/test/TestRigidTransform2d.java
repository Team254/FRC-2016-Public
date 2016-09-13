package com.team254.lib.util.test;

import static org.junit.Assert.*;

import org.junit.Test;

import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class TestRigidTransform2d {
    public static final double kTestEpsilon = 1E-9;

    @Test
    public void testRotation2d() {
        // Test constructors
        Rotation2d rot1 = new Rotation2d();
        assertEquals(1, rot1.cos(), kTestEpsilon);
        assertEquals(0, rot1.sin(), kTestEpsilon);
        assertEquals(0, rot1.tan(), kTestEpsilon);
        assertEquals(0, rot1.getDegrees(), kTestEpsilon);
        assertEquals(0, rot1.getRadians(), kTestEpsilon);

        rot1 = new Rotation2d(1, 1, true);
        assertEquals(Math.sqrt(2) / 2, rot1.cos(), kTestEpsilon);
        assertEquals(Math.sqrt(2) / 2, rot1.sin(), kTestEpsilon);
        assertEquals(1, rot1.tan(), kTestEpsilon);
        assertEquals(45, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 4, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromRadians(Math.PI / 2);
        assertEquals(0, rot1.cos(), kTestEpsilon);
        assertEquals(1, rot1.sin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot1.tan());
        assertEquals(90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(270);
        assertEquals(0, rot1.cos(), kTestEpsilon);
        assertEquals(-1, rot1.sin(), kTestEpsilon);
        System.out.println(rot1.tan());
        assertTrue(-1 / kTestEpsilon > rot1.tan());
        assertEquals(-90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(-Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        // Test inversion
        rot1 = Rotation2d.fromDegrees(270);
        Rotation2d rot2 = rot1.inverse();
        assertEquals(0, rot2.cos(), kTestEpsilon);
        assertEquals(1, rot2.sin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot2.tan());
        assertEquals(90, rot2.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot2.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(1);
        rot2 = rot1.inverse();
        assertEquals(rot1.cos(), rot2.cos(), kTestEpsilon);
        assertEquals(-rot1.sin(), rot2.sin(), kTestEpsilon);
        assertEquals(-1, rot2.getDegrees(), kTestEpsilon);

        // Test rotateBy
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        Rotation2d rot3 = rot1.rotateBy(rot2);
        assertEquals(0, rot3.cos(), kTestEpsilon);
        assertEquals(1, rot3.sin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot3.tan());
        assertEquals(90, rot3.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot3.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        rot3 = rot1.rotateBy(rot2);
        assertEquals(1, rot3.cos(), kTestEpsilon);
        assertEquals(0, rot3.sin(), kTestEpsilon);
        assertEquals(0, rot3.tan(), kTestEpsilon);
        assertEquals(0, rot3.getDegrees(), kTestEpsilon);
        assertEquals(0, rot3.getRadians(), kTestEpsilon);

        // A rotation times its inverse should be the identity
        Rotation2d identity = new Rotation2d();
        rot1 = Rotation2d.fromDegrees(21.45);
        rot2 = rot1.rotateBy(rot1.inverse());
        assertEquals(identity.cos(), rot2.cos(), kTestEpsilon);
        assertEquals(identity.sin(), rot2.sin(), kTestEpsilon);
        assertEquals(identity.getDegrees(), rot2.getDegrees(), kTestEpsilon);

        // Test interpolation
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(135);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(90, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(135);
        rot3 = rot1.interpolate(rot2, .75);
        assertEquals(112.5, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(0, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(45, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(45, rot3.getDegrees(), kTestEpsilon);
    }

    @Test
    public void testTranslation2d() {
        // Test constructors
        Translation2d pos1 = new Translation2d();
        assertEquals(0, pos1.getX(), kTestEpsilon);
        assertEquals(0, pos1.getY(), kTestEpsilon);
        assertEquals(0, pos1.norm(), kTestEpsilon);

        pos1.setX(3);
        pos1.setY(4);
        assertEquals(3, pos1.getX(), kTestEpsilon);
        assertEquals(4, pos1.getY(), kTestEpsilon);
        assertEquals(5, pos1.norm(), kTestEpsilon);

        pos1 = new Translation2d(3, 4);
        assertEquals(3, pos1.getX(), kTestEpsilon);
        assertEquals(4, pos1.getY(), kTestEpsilon);
        assertEquals(5, pos1.norm(), kTestEpsilon);

        // Test inversion
        pos1 = new Translation2d(3.152, 4.1666);
        Translation2d pos2 = pos1.inverse();
        assertEquals(-pos1.getX(), pos2.getX(), kTestEpsilon);
        assertEquals(-pos1.getY(), pos2.getY(), kTestEpsilon);
        assertEquals(pos1.norm(), pos2.norm(), kTestEpsilon);

        // Test rotateBy
        pos1 = new Translation2d(2, 0);
        Rotation2d rot1 = Rotation2d.fromDegrees(90);
        pos2 = pos1.rotateBy(rot1);
        assertEquals(0, pos2.getX(), kTestEpsilon);
        assertEquals(2, pos2.getY(), kTestEpsilon);
        assertEquals(pos1.norm(), pos2.norm(), kTestEpsilon);

        pos1 = new Translation2d(2, 0);
        rot1 = Rotation2d.fromDegrees(-45);
        pos2 = pos1.rotateBy(rot1);
        assertEquals(Math.sqrt(2), pos2.getX(), kTestEpsilon);
        assertEquals(-Math.sqrt(2), pos2.getY(), kTestEpsilon);
        assertEquals(pos1.norm(), pos2.norm(), kTestEpsilon);

        // Test translateBy
        pos1 = new Translation2d(2, 0);
        pos2 = new Translation2d(-2, 1);
        Translation2d pos3 = pos1.translateBy(pos2);
        assertEquals(0, pos3.getX(), kTestEpsilon);
        assertEquals(1, pos3.getY(), kTestEpsilon);
        assertEquals(1, pos3.norm(), kTestEpsilon);

        // A translation times its inverse should be the identity
        Translation2d identity = new Translation2d();
        pos1 = new Translation2d(2.16612, -23.55);
        pos2 = pos1.translateBy(pos1.inverse());
        assertEquals(identity.getX(), pos2.getX(), kTestEpsilon);
        assertEquals(identity.getY(), pos2.getY(), kTestEpsilon);
        assertEquals(identity.norm(), pos2.norm(), kTestEpsilon);

        // Test interpolation
        pos1 = new Translation2d(0, 1);
        pos2 = new Translation2d(10, -1);
        pos3 = pos1.interpolate(pos2, .5);
        assertEquals(5, pos3.getX(), kTestEpsilon);
        assertEquals(0, pos3.getY(), kTestEpsilon);

        pos1 = new Translation2d(0, 1);
        pos2 = new Translation2d(10, -1);
        pos3 = pos1.interpolate(pos2, .75);
        assertEquals(7.5, pos3.getX(), kTestEpsilon);
        assertEquals(-.5, pos3.getY(), kTestEpsilon);
    }

    @Test
    public void testRigidTransform2d() {
        // Test constructors
        RigidTransform2d pose1 = new RigidTransform2d();
        assertEquals(0, pose1.getTranslation().getX(), kTestEpsilon);
        assertEquals(0, pose1.getTranslation().getY(), kTestEpsilon);
        assertEquals(0, pose1.getRotation().getDegrees(), kTestEpsilon);

        pose1 = new RigidTransform2d(new Translation2d(3, 4), Rotation2d.fromDegrees(45));
        assertEquals(3, pose1.getTranslation().getX(), kTestEpsilon);
        assertEquals(4, pose1.getTranslation().getY(), kTestEpsilon);
        assertEquals(45, pose1.getRotation().getDegrees(), kTestEpsilon);

        // Test transformation
        pose1 = new RigidTransform2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        RigidTransform2d pose2 = new RigidTransform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(0));
        RigidTransform2d pose3 = pose1.transformBy(pose2);
        assertEquals(3, pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(5, pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(90, pose3.getRotation().getDegrees(), kTestEpsilon);

        pose1 = new RigidTransform2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new RigidTransform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(-90));
        pose3 = pose1.transformBy(pose2);
        assertEquals(3, pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(5, pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(0, pose3.getRotation().getDegrees(), kTestEpsilon);

        // A pose times its inverse should be the identity
        RigidTransform2d identity = new RigidTransform2d();
        pose1 = new RigidTransform2d(new Translation2d(3.51512152, 4.23), Rotation2d.fromDegrees(91.6));
        pose2 = pose1.transformBy(pose1.inverse());
        assertEquals(identity.getTranslation().getX(), pose2.getTranslation().getX(), kTestEpsilon);
        assertEquals(identity.getTranslation().getY(), pose2.getTranslation().getY(), kTestEpsilon);
        assertEquals(identity.getRotation().getDegrees(), pose2.getRotation().getDegrees(), kTestEpsilon);

        // Test interpolation
        pose1 = new RigidTransform2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new RigidTransform2d(new Translation2d(13, -6), Rotation2d.fromDegrees(-90));
        pose3 = pose1.interpolate(pose2, .5);
        assertEquals(8, pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(-1, pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(0, pose3.getRotation().getDegrees(), kTestEpsilon);

        pose1 = new RigidTransform2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new RigidTransform2d(new Translation2d(13, -6), Rotation2d.fromDegrees(-90));
        pose3 = pose1.interpolate(pose2, .75);
        assertEquals(10.5, pose3.getTranslation().getX(), kTestEpsilon);
        assertEquals(-3.5, pose3.getTranslation().getY(), kTestEpsilon);
        assertEquals(-45, pose3.getRotation().getDegrees(), kTestEpsilon);
    }
}
