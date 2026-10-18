"""Decentralized 3D Reynolds Boids Flocking Controller with Collision Avoidance.

Implements Craig Reynolds' core flocking rules:
1. Separation: Avoid crowding local flockmates (repulsive force)
2. Alignment: Steer towards the average heading of local flockmates
3. Cohesion: Steer towards the average position (center of mass) of flockmates
4. Target Seeking / Migration: Steer towards a shared navigation goal or waypoint
5. Dynamic Obstacle Avoidance: Artificial Potential Field (APF) repulsive velocity
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import numpy as np


def _safe_normalize(vec: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm < eps:
        return np.zeros_like(vec)
    return vec / norm


def _limit_vector(vec: np.ndarray, max_val: float) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm > max_val and norm > 1e-9:
        return (vec / norm) * max_val
    return vec


@dataclass
class BoidConfig:
    """Configurable weights and physics limits for decentralized boids flocking."""
    separation_dist: float = 4.0        # Desired minimum distance between drones (m)
    neighbor_radius: float = 25.0       # Sensing radius for flockmates (m)
    obstacle_radius: float = 8.0        # Detection distance for obstacles (m)
    
    w_separation: float = 2.5           # Separation weight
    w_alignment: float = 1.2            # Alignment weight
    w_cohesion: float = 1.0             # Cohesion weight
    w_target: float = 1.5               # Target seeking weight
    w_obstacle: float = 3.5             # Obstacle avoidance weight
    
    max_speed: float = 8.0              # Max velocity (m/s)
    max_force: float = 4.0              # Max acceleration force (m/s^2)
    damping: float = 0.98               # Velocity damping factor per step
    safety_margin: float = 1.5          # Hard collision boundary (m)


@dataclass
class Obstacle3D:
    """3D Spherical or Cylindrical Obstacle for navigation collision avoidance."""
    position: np.ndarray
    radius: float
    is_dynamic: bool = False
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)


@dataclass
class DroneAgent:
    """Decentralized Drone Agent state in 3D Euclidean space."""
    drone_id: str
    position: np.ndarray
    velocity: np.ndarray
    target: Optional[np.ndarray] = None
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    battery_level: float = 100.0        # Percentage 0-100%
    status: str = "IDLE"                # IDLE, FLYING, RETURNING, EMERGENCY

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float)
        self.velocity = np.asarray(self.velocity, dtype=float)
        if self.target is not None:
            self.target = np.asarray(self.target, dtype=float)


class ReynoldsFlockingController:
    """Decentralized local flocking controller executed independently by each drone."""

    def __init__(self, config: Optional[BoidConfig] = None):
        self.config = config or BoidConfig()

    def compute_separation(self, agent: DroneAgent, neighbors: List[DroneAgent]) -> np.ndarray:
        """Rule 1: Separation - steer away to avoid crowding nearby flockmates."""
        steer = np.zeros(3, dtype=float)
        count = 0

        for other in neighbors:
            if other.drone_id == agent.drone_id:
                continue
            dist = float(np.linalg.norm(agent.position - other.position))
            if 0.0 < dist < self.config.separation_dist:
                diff = agent.position - other.position
                # Normalized and inverse-distance weighted for sharper close-range repulsion
                diff = _safe_normalize(diff) / max(dist, 0.01)
                steer += diff
                count += 1

        if count > 0:
            steer = steer / count
            if np.linalg.norm(steer) > 1e-9:
                steer = _safe_normalize(steer) * self.config.max_speed
                steer = steer - agent.velocity
                steer = _limit_vector(steer, self.config.max_force)

        return steer

    def compute_alignment(self, agent: DroneAgent, neighbors: List[DroneAgent]) -> np.ndarray:
        """Rule 2: Alignment - steer towards average heading of local flockmates."""
        sum_vel = np.zeros(3, dtype=float)
        count = 0

        for other in neighbors:
            if other.drone_id == agent.drone_id:
                continue
            dist = float(np.linalg.norm(agent.position - other.position))
            if 0.0 < dist < self.config.neighbor_radius:
                sum_vel += other.velocity
                count += 1

        if count > 0:
            avg_vel = sum_vel / count
            avg_vel = _safe_normalize(avg_vel) * self.config.max_speed
            steer = avg_vel - agent.velocity
            return _limit_vector(steer, self.config.max_force)

        return np.zeros(3, dtype=float)

    def compute_cohesion(self, agent: DroneAgent, neighbors: List[DroneAgent]) -> np.ndarray:
        """Rule 3: Cohesion - steer towards center of mass of local flockmates."""
        sum_pos = np.zeros(3, dtype=float)
        count = 0

        for other in neighbors:
            if other.drone_id == agent.drone_id:
                continue
            dist = float(np.linalg.norm(agent.position - other.position))
            if 0.0 < dist < self.config.neighbor_radius:
                sum_pos += other.position
                count += 1

        if count > 0:
            center_of_mass = sum_pos / count
            return self._steer_towards(agent, center_of_mass)

        return np.zeros(3, dtype=float)

    def compute_target_seeking(self, agent: DroneAgent) -> np.ndarray:
        """Rule 4: Target Seeking / Migration force."""
        if agent.target is None:
            return np.zeros(3, dtype=float)
        return self._steer_towards(agent, agent.target)

    def compute_obstacle_avoidance(
        self, agent: DroneAgent, obstacles: List[Obstacle3D]
    ) -> np.ndarray:
        """Rule 5: Dynamic Obstacle Avoidance via Artificial Potential Field."""
        steer = np.zeros(3, dtype=float)

        for obs in obstacles:
            diff = agent.position - obs.position
            dist = float(np.linalg.norm(diff))
            effective_radius = obs.radius + self.config.obstacle_radius

            if dist < effective_radius and dist > 1e-4:
                # Strong quadratic repulsion when entering danger zone
                repulsion_mag = (effective_radius - dist) / effective_radius
                repulsive_dir = _safe_normalize(diff)
                
                # Account for dynamic obstacle moving direction
                if obs.is_dynamic and np.linalg.norm(obs.velocity) > 1e-3:
                    relative_vel = agent.velocity - obs.velocity
                    repulsive_dir = _safe_normalize(repulsive_dir + 0.3 * _safe_normalize(relative_vel))

                steer += repulsive_dir * (repulsion_mag ** 2) * self.config.max_force * 2.0

        return _limit_vector(steer, self.config.max_force * 2.5)

    def _steer_towards(self, agent: DroneAgent, target_pos: np.ndarray) -> np.ndarray:
        """Reynolds steering force formula: Desired - Velocity."""
        desired = target_pos - agent.position
        dist = float(np.linalg.norm(desired))
        if dist < 1e-9:
            return -agent.velocity

        # Arrival slowing radius
        slowing_radius = 5.0
        if dist < slowing_radius:
            speed = self.config.max_speed * (dist / slowing_radius)
        else:
            speed = self.config.max_speed

        desired = (desired / dist) * speed
        steer = desired - agent.velocity
        return _limit_vector(steer, self.config.max_force)

    def compute_step_acceleration(
        self,
        agent: DroneAgent,
        neighbors: List[DroneAgent],
        obstacles: Optional[List[Obstacle3D]] = None,
    ) -> np.ndarray:
        """Combines all flocking force vectors according to priority and weights."""
        obstacles = obstacles or []

        f_sep = self.compute_separation(agent, neighbors) * self.config.w_separation
        f_ali = self.compute_alignment(agent, neighbors) * self.config.w_alignment
        f_coh = self.compute_cohesion(agent, neighbors) * self.config.w_cohesion
        f_tgt = self.compute_target_seeking(agent) * self.config.w_target
        f_obs = self.compute_obstacle_avoidance(agent, obstacles) * self.config.w_obstacle

        total_force = f_sep + f_ali + f_coh + f_tgt + f_obs
        return _limit_vector(total_force, self.config.max_force)


class SwarmSimulator:
    """High-performance 3D multi-agent discrete-time physics simulator."""

    def __init__(
        self,
        agents: Optional[List[DroneAgent]] = None,
        obstacles: Optional[List[Obstacle3D]] = None,
        config: Optional[BoidConfig] = None,
    ):
        self.config = config or BoidConfig()
        self.controller = ReynoldsFlockingController(self.config)
        self.agents: Dict[str, DroneAgent] = {}
        if agents:
            for a in agents:
                self.agents[a.drone_id] = a
        self.obstacles: List[Obstacle3D] = obstacles or []

    def add_agent(self, agent: DroneAgent) -> None:
        self.agents[agent.drone_id] = agent

    def add_obstacle(self, obstacle: Obstacle3D) -> None:
        self.obstacles.append(obstacle)

    def step(self, dt: float = 0.1) -> None:
        """Advance the physics simulation by dt seconds with decentralized evaluation."""
        agent_list = list(self.agents.values())
        accelerations = {}

        # 1. Compute forces for each agent based on state at t
        for agent in agent_list:
            acc = self.controller.compute_step_acceleration(agent, agent_list, self.obstacles)
            accelerations[agent.drone_id] = acc

        # 2. Integrate velocities and positions (Semi-implicit Euler)
        for agent in agent_list:
            agent.acceleration = accelerations[agent.drone_id]
            agent.velocity += agent.acceleration * dt
            agent.velocity *= self.config.damping
            agent.velocity = _limit_vector(agent.velocity, self.config.max_speed)
            agent.position += agent.velocity * dt

        # 3. Update dynamic obstacles
        for obs in self.obstacles:
            if obs.is_dynamic:
                obs.position += obs.velocity * dt

    def get_swarm_center(self) -> np.ndarray:
        if not self.agents:
            return np.zeros(3, dtype=float)
        positions = np.array([a.position for a in self.agents.values()])
        return np.mean(positions, axis=0)

    def get_swarm_velocity(self) -> np.ndarray:
        if not self.agents:
            return np.zeros(3, dtype=float)
        velocities = np.array([a.velocity for a in self.agents.values()])
        return np.mean(velocities, axis=0)

    def get_minimum_pairwise_distance(self) -> float:
        """Calculates closest proximity between any two agents in the swarm."""
        positions = [a.position for a in self.agents.values()]
        n = len(positions)
        if n < 2:
            return float("inf")
        min_dist = float("inf")
        for i in range(n):
            for j in range(i + 1, n):
                d = float(np.linalg.norm(positions[i] - positions[j]))
                if d < min_dist:
                    min_dist = d
        return min_dist

    def get_metrics(self) -> Dict[str, float]:
        """Swarm coherence and alignment metrics."""
        positions = np.array([a.position for a in self.agents.values()])
        velocities = np.array([a.velocity for a in self.agents.values()])
        center = np.mean(positions, axis=0)
        
        # Cohesion metric: average distance to center of mass
        cohesion = float(np.mean([np.linalg.norm(p - center) for p in positions]))
        
        # Polarization metric: alignment of headings (0 to 1)
        speed_norms = np.linalg.norm(velocities, axis=1, keepdims=True)
        valid_indices = np.where(speed_norms.squeeze() > 1e-4)[0]
        if len(valid_indices) > 0:
            unit_vels = velocities[valid_indices] / speed_norms[valid_indices]
            polarization = float(np.linalg.norm(np.mean(unit_vels, axis=0)))
        else:
            polarization = 1.0

        return {
            "num_agents": len(self.agents),
            "swarm_radius": cohesion,
            "polarization": polarization,
            "min_distance": self.get_minimum_pairwise_distance(),
        }


def main():
    """CLI demonstration of 3D swarm flocking."""
    import sys
    print("=== Swarm Nav Mesh - Decentralized Boids Flocking Simulation ===")
    config = BoidConfig(w_separation=3.0, w_alignment=1.5, w_cohesion=1.2, w_target=2.0)
    sim = SwarmSimulator(config=config)

    # Spawn 8 drones in a scattered cluster
    np.random.seed(42)
    target = np.array([50.0, 50.0, 20.0])
    for i in range(8):
        pos = np.random.uniform(-10, 10, size=3) + np.array([0.0, 0.0, 15.0])
        vel = np.random.uniform(-1, 1, size=3)
        sim.add_agent(DroneAgent(drone_id=f"drone-{i:02d}", position=pos, velocity=vel, target=target))

    # Add an obstacle between swarm and target
    sim.add_obstacle(Obstacle3D(position=np.array([25.0, 25.0, 18.0]), radius=5.0))

    print("Simulating 50 steps...")
    for t in range(50):
        sim.step(dt=0.1)
        if t % 10 == 0:
            metrics = sim.get_metrics()
            center = sim.get_swarm_center()
            print(f"Step {t:02d}: Center=({center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f}), "
                  f"Radius={metrics['swarm_radius']:.2f}m, MinDist={metrics['min_distance']:.2f}m, "
                  f"Polarization={metrics['polarization']:.2f}")


if __name__ == "__main__":
    main()
