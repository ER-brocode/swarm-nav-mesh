"""Test suite for 3D Waypoint Trajectory Planning and Geofencing Enforcement."""

import numpy as np
import pytest

from telemetry.planner import (
    GeofenceEnforcer,
    GeofenceType,
    GeofenceVolume,
    TrajectoryPlanner3D,
    Waypoint3D,
)


class TestTelemetryPlanner:
    """Tests trajectory interpolation and boundary enforcement."""

    def test_trajectory_interpolation_generation(self):
        planner = TrajectoryPlanner3D(cruise_speed=5.0)
        waypoints = [
            Waypoint3D(0.0, 0.0, 0.0),
            Waypoint3D(10.0, 0.0, 0.0),
            Waypoint3D(10.0, 10.0, 5.0),
        ]
        path = planner.generate_interpolated_path(waypoints, resolution=1.0)
        assert len(path) > 10
        # Check start and end proximity
        assert np.allclose(path[0], [0.0, 0.0, 0.0], atol=1e-3)
        assert np.allclose(path[-1], [10.0, 10.0, 5.0], atol=1e-3)

    def test_flight_duration_calculation(self):
        planner = TrajectoryPlanner3D()
        waypoints = [
            Waypoint3D(0.0, 0.0, 0.0, speed=10.0, dwell_time=2.0),
            Waypoint3D(50.0, 0.0, 0.0, speed=10.0, dwell_time=1.0),
        ]
        duration = planner.compute_flight_duration(waypoints)
        # 50m / 10m/s = 5s + 2s dwell = 7.0s
        assert abs(duration - 7.0) < 1e-4

    def test_inclusion_geofence_clamping(self):
        enforcer = GeofenceEnforcer(
            inclusion_zone=GeofenceVolume(
                fence_id="safe_box",
                fence_type=GeofenceType.INCLUSION_AABB,
                min_bound=np.array([-10.0, -10.0, 0.0]),
                max_bound=np.array([10.0, 10.0, 20.0]),
            )
        )

        outside_pt = np.array([15.0, -12.0, 25.0])
        clamped, violations = enforcer.validate_and_clamp(outside_pt)

        assert len(violations) == 1
        assert violations[0].fence_id == "safe_box"
        assert np.array_equal(clamped, np.array([10.0, -10.0, 20.0]))

    def test_exclusion_cylinder_radial_push(self):
        enforcer = GeofenceEnforcer()
        enforcer.add_exclusion_zone(
            GeofenceVolume(
                fence_id="restricted_no_fly",
                fence_type=GeofenceType.EXCLUSION_CYLINDER,
                center=np.array([0.0, 0.0]),
                radius=10.0,
                floor_z=0.0,
                ceiling_z=50.0,
            )
        )

        inside_pt = np.array([3.0, 4.0, 15.0])  # hypot(3, 4) = 5.0 < radius 10.0
        clamped, violations = enforcer.validate_and_clamp(inside_pt)

        assert len(violations) == 1
        assert violations[0].fence_id == "restricted_no_fly"
        dist_from_center = np.hypot(clamped[0], clamped[1])
        # Pushed outside radius (10.0 + 1.0 margin = 11.0)
        assert dist_from_center >= 10.0
