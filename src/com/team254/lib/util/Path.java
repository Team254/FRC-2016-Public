package com.team254.lib.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * A Path is a recording of the path that the robot takes. Path objects consist
 * of a List of Waypoints that the robot passes by. Using multiple Waypoints in
 * a Path object and the robot's current speed, the code can extrapolate future
 * Waypoints and predict the robot's motion. It can also dictate the robot's
 * motion along the set path.
 */
public class Path {
    protected static final double kSegmentCompletePercentage = .99;

    protected List<Waypoint> mWaypoints;
    protected List<PathSegment> mSegments;
    protected Set<String> mMarkersCrossed;

    /**
     * A point along the Path, which consists of the location, the speed, and a
     * string marker (that future code can identify). Paths consist of a List of
     * Waypoints.
     */
    public static class Waypoint {
        public final Translation2d position;
        public final double speed;
        public final Optional<String> marker;

        public Waypoint(Translation2d position, double speed) {
            this.position = position;
            this.speed = speed;
            this.marker = Optional.empty();
        }

        public Waypoint(Translation2d position, double speed, String marker) {
            this.position = position;
            this.speed = speed;
            this.marker = Optional.of(marker);
        }
    }

    public Path(List<Waypoint> waypoints) {
        mMarkersCrossed = new HashSet<String>();
        mWaypoints = waypoints;
        mSegments = new ArrayList<PathSegment>();
        for (int i = 0; i < waypoints.size() - 1; ++i) {
            mSegments.add(
                    new PathSegment(waypoints.get(i).position, waypoints.get(i + 1).position, waypoints.get(i).speed));
        }
        // The first waypoint is already complete
        if (mWaypoints.size() > 0) {
            Waypoint first_waypoint = mWaypoints.get(0);
            if (first_waypoint.marker.isPresent()) {
                mMarkersCrossed.add(first_waypoint.marker.get());
            }
            mWaypoints.remove(0);
        }
    }

    /**
     * @param An
     *            initial position
     * @return Returns the distance from the position to the first point on the
     *         path
     */
    public double update(Translation2d position) {
        double rv = 0.0;
        for (Iterator<PathSegment> it = mSegments.iterator(); it.hasNext();) {
            PathSegment segment = it.next();
            PathSegment.ClosestPointReport closest_point_report = segment.getClosestPoint(position);
            if (closest_point_report.index >= kSegmentCompletePercentage) {
                it.remove();
                if (mWaypoints.size() > 0) {
                    Waypoint waypoint = mWaypoints.get(0);
                    if (waypoint.marker.isPresent()) {
                        mMarkersCrossed.add(waypoint.marker.get());
                    }
                    mWaypoints.remove(0);
                }
            } else {
                if (closest_point_report.index > 0.0) {
                    // Can shorten this segment
                    segment.updateStart(closest_point_report.closest_point);
                }
                // We are done
                rv = closest_point_report.distance;
                // ...unless the next segment is closer now
                if (it.hasNext()) {
                    PathSegment next = it.next();
                    PathSegment.ClosestPointReport next_closest_point_report = next.getClosestPoint(position);
                    if (next_closest_point_report.index > 0
                            && next_closest_point_report.index < kSegmentCompletePercentage
                            && next_closest_point_report.distance < rv) {
                        next.updateStart(next_closest_point_report.closest_point);
                        rv = next_closest_point_report.distance;
                        mSegments.remove(0);
                        if (mWaypoints.size() > 0) {
                            Waypoint waypoint = mWaypoints.get(0);
                            if (waypoint.marker.isPresent()) {
                                mMarkersCrossed.add(waypoint.marker.get());
                            }
                            mWaypoints.remove(0);
                        }
                    }
                }
                break;
            }
        }
        return rv;
    }

    public Set<String> getMarkersCrossed() {
        return mMarkersCrossed;
    }

    public double getRemainingLength() {
        double length = 0.0;
        for (int i = 0; i < mSegments.size(); ++i) {
            length += mSegments.get(i).getLength();
        }
        return length;
    }

    /**
     * @param The
     *            robot's current position
     * @param A
     *            specified distance to predict a future waypoint
     * @return A segment of the robot's predicted motion with start/end points
     *         and speed.
     */
    public PathSegment.Sample getLookaheadPoint(Translation2d position, double lookahead_distance) {
        if (mSegments.size() == 0) {
            return new PathSegment.Sample(new Translation2d(), 0);
        }

        // Check the distances to the start and end of each segment. As soon as
        // we find a point > lookahead_distance away, we know the right point
        // lies somewhere on that segment.
        Translation2d position_inverse = position.inverse();
        if (position_inverse.translateBy(mSegments.get(0).getStart()).norm() >= lookahead_distance) {
            // Special case: Before the first point, so just return the first
            // point.
            return new PathSegment.Sample(mSegments.get(0).getStart(), mSegments.get(0).getSpeed());
        }
        for (int i = 0; i < mSegments.size(); ++i) {
            PathSegment segment = mSegments.get(i);
            double distance = position_inverse.translateBy(segment.getEnd()).norm();
            if (distance >= lookahead_distance) {
                // This segment contains the lookahead point
                Optional<Translation2d> intersection_point = getFirstCircleSegmentIntersection(segment, position,
                        lookahead_distance);
                if (intersection_point.isPresent()) {
                    return new PathSegment.Sample(intersection_point.get(), segment.getSpeed());
                } else {
                    System.out.println("ERROR: No intersection point?");
                }
            }
        }
        // Special case: After the last point, so extrapolate forward.
        PathSegment last_segment = mSegments.get(mSegments.size() - 1);
        PathSegment new_last_segment = new PathSegment(last_segment.getStart(), last_segment.interpolate(10000),
                last_segment.getSpeed());
        Optional<Translation2d> intersection_point = getFirstCircleSegmentIntersection(new_last_segment, position,
                lookahead_distance);
        if (intersection_point.isPresent()) {
            return new PathSegment.Sample(intersection_point.get(), last_segment.getSpeed());
        } else {
            System.out.println("ERROR: No intersection point anywhere on line?");
            return new PathSegment.Sample(last_segment.getEnd(), last_segment.getSpeed());
        }
    }

    static Optional<Translation2d> getFirstCircleSegmentIntersection(PathSegment segment, Translation2d center,
            double radius) {
        double x1 = segment.getStart().getX() - center.getX();
        double y1 = segment.getStart().getY() - center.getY();
        double x2 = segment.getEnd().getX() - center.getX();
        double y2 = segment.getEnd().getY() - center.getY();
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr_squared = dx * dx + dy * dy;
        double det = x1 * y2 - x2 * y1;

        double discriminant = dr_squared * radius * radius - det * det;
        if (discriminant < 0) {
            // No intersection
            return Optional.empty();
        }

        double sqrt_discriminant = Math.sqrt(discriminant);
        Translation2d pos_solution = new Translation2d(
                (det * dy + (dy < 0 ? -1 : 1) * dx * sqrt_discriminant) / dr_squared + center.getX(),
                (-det * dx + Math.abs(dy) * sqrt_discriminant) / dr_squared + center.getY());
        Translation2d neg_solution = new Translation2d(
                (det * dy - (dy < 0 ? -1 : 1) * dx * sqrt_discriminant) / dr_squared + center.getX(),
                (-det * dx - Math.abs(dy) * sqrt_discriminant) / dr_squared + center.getY());

        // Choose the one between start and end that is closest to start
        double pos_dot_product = segment.dotProduct(pos_solution);
        double neg_dot_product = segment.dotProduct(neg_solution);
        if (pos_dot_product < 0 && neg_dot_product >= 0) {
            return Optional.of(neg_solution);
        } else if (pos_dot_product >= 0 && neg_dot_product < 0) {
            return Optional.of(pos_solution);
        } else {
            if (Math.abs(pos_dot_product) <= Math.abs(neg_dot_product)) {
                return Optional.of(pos_solution);
            } else {
                return Optional.of(neg_solution);
            }
        }
    }
}
