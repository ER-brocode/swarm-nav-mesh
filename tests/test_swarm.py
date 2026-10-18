"""Test suite for Swarm Reynolds Boids Flocking, Formation Convergence, and Collision Avoidance."""

import math
import numpy as np
import pytest

from swarm.flocking import (
    BoidConfig,
    DroneAgent,
    Obstacle3D,
    ReynoldsFlockingController,
    SwarmSimulator,
)


class TestReynoldsFlocking:
    """Tests 3D Reynolds boids flocking rules, convergence, and obstacle avoidance."""

    def test_separation_force_repels_close_neighbors(self):
        config = BoidConfig(separation_dist=5.0, w_separation=2.0)
        controller = ReynoldsFlockingController(config)

        # Agent at origin, neighbor very close at (1, 0, 0)
        agent = DroneAgent("d1", position=np.array([0.0, 0.0, 0.0]), velocity=np.array([0.0, 0.0, 0.0]))
        neighbor = DroneAgent("d2", position=np.array([1.0, 0.0, 0.0]), velocity=np.array([0.0, 0.0, 0.0]))

        sep_force = controller.compute_separation(agent, [neighbor])
        # Separation force must point away from neighbor (negative X direction)
        assert sep_force[0] < -0.1
        assert abs(sep_force[1]) < 1e-4
        assert abs(sep_force[2]) < 1e-4

    def test_alignment_matches_neighbor_velocity(self):
        config = BoidConfig(neighbor_radius=20.0, max_speed=5.0)
        controller = ReynoldsFlockingController(config)

        agent = DroneAgent("d1", position=np.array([0.0, 0.0, 0.0]), velocity=np.array([0.0, 1.0, 0.0]))
        neighbor = DroneAgent("d2", position=np.array([2.0, 0.0, 0.0]), velocity=np.array([5.0, 0.0, 0.0]))

        align_force = controller.compute_alignment(agent, [neighbor])
        # Force must steer agent heading towards positive X
        assert align_force[0] > 0.0

    def test_cohesion_steers_to_center_of_mass(self):
        config = BoidConfig(neighbor_radius=30.0, max_speed=5.0)
        controller = ReynoldsFlockingController(config)

        agent = DroneAgent("d1", position=np.array([0.0, 0.0, 0.0]), velocity=np.array([0.0, 0.0, 0.0]))
        n1 = DroneAgent("d2", position=np.array([10.0, 0.0, 0.0]), velocity=np.array([0.0, 0.0, 0.0]))
        n2 = DroneAgent("d3", position=np.array([10.0, 10.0, 0.0]), velocity=np.array([0.0, 0.0, 0.0]))

        coh_force = controller.compute_cohesion(agent, [n1, n2])
        # Center of mass is (10, 5, 0), so force should be positive in X and Y
        assert coh_force[0] > 0.0
        assert coh_force[1] > 0.0

    def test_collision_avoidance_with_obstacle(self):
        config = BoidConfig(obstacle_radius=10.0, max_force=5.0)
        controller = ReynoldsFlockingController(config)

        agent = DroneAgent("d1", position=np.array([0.0, 0.0, 0.0]), velocity=np.array([5.0, 0.0, 0.0]))
        # Obstacle placed directly ahead at (5, 0, 0)
        obs = Obstacle3D(position=np.array([5.0, 0.0, 0.0]), radius=2.0)

        obs_force = controller.compute_obstacle_avoidance(agent, [obs])
        # Force must repel agent away from obstacle (negative X direction)
        assert obs_force[0] < -0.1

    def test_swarm_formation_convergence(self):
        """Simulate 6 drones converging on a common target without colliding."""
        config = BoidConfig(
            separation_dist=3.0,
            neighbor_radius=30.0,
            w_separation=3.0,
            w_alignment=1.5,
            w_cohesion=1.0,
            w_target=2.0,
            max_speed=6.0,
        )
        sim = SwarmSimulator(config=config)
        target = np.array([40.0, 40.0, 20.0])

        # Initialize scattered drones
        initial_positions = [
            [-10.0, -5.0, 5.0],
            [-15.0, 0.0, 5.0],
            [-5.0, -10.0, 5.0],
            [-8.0, 8.0, 5.0],
            [-12.0, -12.0, 5.0],
            [-2.0, 2.0, 5.0],
        ]
        for i, pos in enumerate(initial_positions):
            sim.add_agent(DroneAgent(
                drone_id=f"drone_{i}",
                position=np.array(pos, dtype=float),
                velocity=np.zeros(3, dtype=float),
                target=target,
            ))

        # Initial distance to target
        initial_center = sim.get_swarm_center()
        initial_dist = float(np.linalg.norm(initial_center - target))

        # Run simulation for 80 steps
        for _ in range(80):
            sim.step(dt=0.1)
            # Ensure safety margin is never violated
            min_dist = sim.get_minimum_pairwise_distance()
            assert min_dist >= 0.5, f"Collision detected! Min distance: {min_dist}m"

        final_center = sim.get_swarm_center()
        final_dist = float(np.linalg.norm(final_center - target))

        # Swarm must have converged significantly closer to the navigation target
        assert final_dist < initial_dist * 0.5, f"Swarm failed to converge: initial={initial_dist}, final={final_dist}"
        
        # Swarm must maintain high polarization (cohesive direction)
        metrics = sim.get_metrics()
        assert metrics["polarization"] > 0.5
