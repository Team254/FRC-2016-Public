package com.team254.lib.util.test;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.junit.Test;

import com.team254.lib.util.AdaptivePurePursuitController;
import com.team254.lib.util.Path;
import com.team254.lib.util.AdaptivePurePursuitController.Circle;
import com.team254.lib.util.Path.Waypoint;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class AdaptivePurePursuitControllerTest {
    private static final double kEpsilon = 1E-9;

    @Test
    public void testJoinPath() {
        // Robot is at the origin
        RigidTransform2d robot_pose = new RigidTransform2d();

        // Lookahead point is at the same point
        Translation2d lookahead_point = new Translation2d(0, 0);

        // Should be null
        Optional<Circle> circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertFalse(circle.isPresent());

        // Lookahead point is forward
        lookahead_point = new Translation2d(1, 0);
        circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertFalse(circle.isPresent());

        robot_pose = new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90));
        lookahead_point = new Translation2d(0, 1);
        circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertFalse(circle.isPresent());

        // Lookahead point is 1m to the left
        robot_pose = new RigidTransform2d();
        lookahead_point = new Translation2d(0, 1);
        circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertTrue(circle.isPresent());
        assertEquals(.5, circle.get().radius, kEpsilon);
        assertEquals(0, circle.get().center.getX(), kEpsilon);
        assertEquals(.5, circle.get().center.getY(), kEpsilon);
        assertFalse(circle.get().turn_right);

        // 1m to the right
        lookahead_point = new Translation2d(0, -1);
        circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertTrue(circle.isPresent());
        assertEquals(.5, circle.get().radius, kEpsilon);
        assertEquals(0, circle.get().center.getX(), kEpsilon);
        assertEquals(-.5, circle.get().center.getY(), kEpsilon);
        assertTrue(circle.get().turn_right);

        // Lookahead point is forward, robot pose is rotated
        robot_pose = new RigidTransform2d(new Translation2d(1, 1), Rotation2d.fromDegrees(45));
        lookahead_point = new Translation2d(2, 2);
        circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertFalse(circle.isPresent());

        robot_pose = new RigidTransform2d(new Translation2d(1, 1), Rotation2d.fromDegrees(-45));
        lookahead_point = new Translation2d(2, 0);
        circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertFalse(circle.isPresent());

        robot_pose = new RigidTransform2d(new Translation2d(1, 1), Rotation2d.fromDegrees(135));
        lookahead_point = new Translation2d(0, 2);
        circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertFalse(circle.isPresent());

        robot_pose = new RigidTransform2d(new Translation2d(1, 1), Rotation2d.fromDegrees(-135));
        lookahead_point = new Translation2d(2, 2);
        circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertFalse(circle.isPresent());

        robot_pose = RigidTransform2d.fromTranslation(new Translation2d(1.7, 0));
        lookahead_point = new Translation2d(2, .4);
        circle = AdaptivePurePursuitController.joinPath(robot_pose, lookahead_point);
        assertTrue(circle.isPresent());
        assertEquals(1.7, circle.get().center.getX(), kEpsilon);
        assertEquals(circle.get().center.getY(), circle.get().radius, kEpsilon);
        assertFalse(circle.get().turn_right);

    }

    @Test
    public void testController() {
        List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Translation2d(0, 0), 1));
        waypoints.add(new Waypoint(new Translation2d(1, 0), 1));
        waypoints.add(new Waypoint(new Translation2d(2, 0), 2));
        waypoints.add(new Waypoint(new Translation2d(2, -1), 2));
        waypoints.add(new Waypoint(new Translation2d(2, -2), 1));
        waypoints.add(new Waypoint(new Translation2d(3, -2), 1));
        waypoints.add(new Waypoint(new Translation2d(4, -2), 1));
        waypoints.add(new Waypoint(new Translation2d(5, -2), 1));
        Path path = new Path(waypoints);

        double dt = .01;
        AdaptivePurePursuitController controller = new AdaptivePurePursuitController(0.25, 1.0, dt, path, false, 0);

        RigidTransform2d robot_pose = new RigidTransform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));
        double t = 0;
        while (!controller.isDone() && t < 10) {
            // Follow the path
            RigidTransform2d.Delta command = controller.update(robot_pose, t);
            robot_pose = robot_pose.transformBy(new RigidTransform2d(new Translation2d(command.dx * dt, 0),
                    Rotation2d.fromRadians(command.dtheta * dt)));

            System.out.println(
                    "t = " + t + ", lin vel " + command.dx + ", ang vel " + command.dtheta + ", pose " + robot_pose);
            t += dt;
        }
        assertTrue(controller.isDone());
        assertEquals(5, robot_pose.getTranslation().getX(), .01);
        assertEquals(-2, robot_pose.getTranslation().getY(), .01);
    }

    @Test
    public void testControllerReversed() {
        List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Translation2d(0, 0), 1));
        waypoints.add(new Waypoint(new Translation2d(1, 0), 1));
        waypoints.add(new Waypoint(new Translation2d(2, 0), 2, "StartedTurn"));
        waypoints.add(new Waypoint(new Translation2d(2, -1), 2));
        waypoints.add(new Waypoint(new Translation2d(2, -2), 1, "FinishedTurn"));
        waypoints.add(new Waypoint(new Translation2d(3, -2), 1));
        waypoints.add(new Waypoint(new Translation2d(4, -2), 3));
        waypoints.add(new Waypoint(new Translation2d(5, -2), 1));
        Path path = new Path(waypoints);

        double dt = .01;
        AdaptivePurePursuitController controller = new AdaptivePurePursuitController(0.25, 1.0, dt, path, true,
                kEpsilon);

        RigidTransform2d robot_pose = RigidTransform2d.fromRotation(Rotation2d.fromRadians(Math.PI));
        double t = 0;
        while (!controller.isDone() && t < 10) {
            // Follow the path
            RigidTransform2d.Delta command = controller.update(robot_pose, t);
            robot_pose = robot_pose.transformBy(new RigidTransform2d(new Translation2d(command.dx * dt, 0),
                    Rotation2d.fromRadians(command.dtheta * dt)));

            System.out.println(
                    "t = " + t + ", lin vel " + command.dx + ", ang vel " + command.dtheta + ", pose " + robot_pose);
            t += dt;
        }
        assertTrue(controller.isDone());
        assertEquals(2, controller.getMarkersCrossed().size());
        assertEquals(5, robot_pose.getTranslation().getX(), .01);
        assertEquals(-2, robot_pose.getTranslation().getY(), .01);
    }
}
