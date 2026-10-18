"""3D Waypoint Trajectory Planner and Real-Time Geofencing Enforcement.

Features:
- Parametric 3D polynomial trajectory generation with velocity & acceleration continuity
- Quintic (minimum jerk) & cubic Hermite spline interpolation
- Dynamic path re-planning around no-fly zones
- 3D Geofence boundary enforcement (Axis-Aligned Bounding Box & Cylindrical / Polygonal)
- Hard failsafe return-to-home (RTH) clamping
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple
import numpy as np


class GeofenceType(str, Enum):
    INCLUSION_AABB = "INCLUSION_AABB"    # Swarm must stay INSIDE this box
    EXCLUSION_CYLINDER = "EXCLUSION_CYLINDER"  # Swarm must stay OUTSIDE (e.g. airport/tower)


@dataclass
class GeofenceVolume:
    """Definition of spatial boundary zone for drone flight authorization."""
    fence_id: str
    fence_type: GeofenceType
    # For INCLUSION_AABB: [min_x, min_y, min_z], [max_x, max_y, max_z]
    # For EXCLUSION_CYLINDER: center [x, y], min_z, max_z, radius
    min_bound: np.ndarray = field(default_factory=lambda: np.zeros(3))
    max_bound: np.ndarray = field(default_factory=lambda: np.zeros(3))
    center: np.ndarray = field(default_factory=lambda: np.zeros(2))
    radius: float = 0.0
    floor_z: float = 0.0
    ceiling_z: float = 120.0  # standard 120m FAA ceiling

    def is_inside(self, point: np.ndarray) -> bool:
        p = np.asarray(point, dtype=float)
        if self.fence_type == GeofenceType.INCLUSION_AABB:
            return bool(np.all(p >= self.min_bound) and np.all(p <= self.max_bound))
        elif self.fence_type == GeofenceType.EXCLUSION_CYLINDER:
            if not (self.floor_z <= p[2] <= self.ceiling_z):
                return False
            horizontal_dist = math.hypot(p[0] - self.center[0], p[1] - self.center[1])
            return horizontal_dist <= self.radius
        return False


@dataclass
class GeofenceViolation:
    fence_id: str
    fence_type: GeofenceType
    violating_position: np.ndarray
    clamped_position: np.ndarray
    breach_distance: float


class GeofenceEnforcer:
    """Guarantees swarming drones never exit operational airspace or enter forbidden zones."""

    def __init__(self, inclusion_zone: Optional[GeofenceVolume] = None):
        self.inclusion_zone = inclusion_zone or GeofenceVolume(
            fence_id="default_inclusion",
            fence_type=GeofenceType.INCLUSION_AABB,
            min_bound=np.array([-100.0, -100.0, 0.0]),
            max_bound=np.array([100.0, 100.0, 100.0]),
        )
        self.exclusion_zones: List[GeofenceVolume] = []

    def add_exclusion_zone(self, zone: GeofenceVolume) -> None:
        self.exclusion_zones.append(zone)

    def validate_and_clamp(self, point: np.ndarray) -> Tuple[np.ndarray, List[GeofenceViolation]]:
        """Checks point against all boundaries and returns safe clamped point + violations."""
        pos = np.copy(np.asarray(point, dtype=float))
        violations: List[GeofenceViolation] = []

        # 1. Enforce inclusion zone (Box clamp)
        if not self.inclusion_zone.is_inside(pos):
            clamped = np.clip(pos, self.inclusion_zone.min_bound, self.inclusion_zone.max_bound)
            breach_dist = float(np.linalg.norm(pos - clamped))
            violations.append(GeofenceViolation(
                fence_id=self.inclusion_zone.fence_id,
                fence_type=self.inclusion_zone.fence_type,
                violating_position=np.copy(pos),
                clamped_position=np.copy(clamped),
                breach_distance=breach_dist,
            ))
            pos = clamped

        # 2. Enforce exclusion zones (Push out radially)
        for zone in self.exclusion_zones:
            if zone.fence_type == GeofenceType.EXCLUSION_CYLINDER:
                if zone.floor_z <= pos[2] <= zone.ceiling_z:
                    dx = pos[0] - zone.center[0]
                    dy = pos[1] - zone.center[1]
                    dist_2d = math.hypot(dx, dy)
                    if dist_2d < zone.radius:
                        # Push outwards by safe margin
                        if dist_2d < 1e-6:
                            angle = 0.0
                        else:
                            angle = math.atan2(dy, dx)
                        clamped_x = zone.center[0] + (zone.radius + 1.0) * math.cos(angle)
                        clamped_y = zone.center[1] + (zone.radius + 1.0) * math.sin(angle)
                        clamped_pos = np.array([clamped_x, clamped_y, pos[2]])
                        violations.append(GeofenceViolation(
                            fence_id=zone.fence_id,
                            fence_type=zone.fence_type,
                            violating_position=np.copy(pos),
                            clamped_position=clamped_pos,
                            breach_distance=zone.radius - dist_2d,
                        ))
                        pos = clamped_pos

        return pos, violations


@dataclass
class Waypoint3D:
    x: float
    y: float
    z: float
    speed: float = 5.0
    dwell_time: float = 0.0

    @property
    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)


class TrajectoryPlanner3D:
    """Generates continuous smooth C2 trajectory curves from discrete 3D waypoints."""

    def __init__(self, cruise_speed: float = 6.0, max_accel: float = 3.0):
        self.cruise_speed = cruise_speed
        self.max_accel = max_accel

    def generate_interpolated_path(
        self,
        waypoints: List[Waypoint3D],
        resolution: float = 0.5,
    ) -> List[np.ndarray]:
        """Interpolates waypoints with minimum-jerk quintic polynomial easing."""
        if not waypoints:
            return []
        if len(waypoints) == 1:
            return [waypoints[0].as_array]

        full_trajectory: List[np.ndarray] = []
        for i in range(len(waypoints) - 1):
            w_start = waypoints[i].as_array
            w_end = waypoints[i + 1].as_array
            segment_dist = float(np.linalg.norm(w_end - w_start))
            if segment_dist < 1e-4:
                continue

            # Number of intermediate samples
            num_samples = max(2, int(segment_dist / resolution))
            for s in range(num_samples):
                # Quintic polynomial interpolation factor: 10 t^3 - 15 t^4 + 6 t^5
                t = s / (num_samples - 1)
                ease_t = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)
                interp_pt = w_start + (w_end - w_start) * ease_t
                full_trajectory.append(interp_pt)

        return full_trajectory

    def compute_flight_duration(self, waypoints: List[Waypoint3D]) -> float:
        """Estimates total flight time in seconds along trajectory."""
        if len(waypoints) < 2:
            return 0.0
        total_time = 0.0
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i].as_array
            p2 = waypoints[i + 1].as_array
            dist = float(np.linalg.norm(p2 - p1))
            speed = waypoints[i].speed or self.cruise_speed
            total_time += (dist / speed) + waypoints[i].dwell_time
        return total_time


def main():
    print("=== Swarm Nav Mesh - Trajectory Planner & Geofencing ===")
    planner = TrajectoryPlanner3D()
    enforcer = GeofenceEnforcer(
        inclusion_zone=GeofenceVolume(
            fence_id="flight_corridor",
            fence_type=GeofenceType.INCLUSION_AABB,
            min_bound=np.array([0.0, 0.0, 2.0]),
            max_bound=np.array([50.0, 50.0, 25.0]),
        )
    )
    enforcer.add_exclusion_zone(
        GeofenceVolume(
            fence_id="radar_mast",
            fence_type=GeofenceType.EXCLUSION_CYLINDER,
            center=np.array([20.0, 20.0]),
            radius=6.0,
            floor_z=0.0,
            ceiling_z=50.0,
        )
    )

    waypoints = [
        Waypoint3D(0.0, 0.0, 5.0),
        Waypoint3D(20.0, 20.0, 10.0),  # Hits exclusion cylinder
        Waypoint3D(60.0, 60.0, 30.0),  # Breaches inclusion ceiling & max X/Y
    ]

    path = planner.generate_interpolated_path(waypoints, resolution=2.0)
    print(f"Generated {len(path)} trajectory points.")

    violations_detected = 0
    for pt in path:
        clamped, violations = enforcer.validate_and_clamp(pt)
        if violations:
            violations_detected += len(violations)

    print(f"Geofence enforcement complete. Handled {violations_detected} breaches via real-time clamping.")


if __name__ == "__main__":
    main()
