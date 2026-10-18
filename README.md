# 🛸 Swarm Nav Mesh (Autonomous Drone Swarm Telemetry & Navigation Mesh)

[![CI / Swarm Mesh Test Suite](https://github.com/ER-brocode/swarm-nav-mesh/actions/workflows/ci.yml/badge.svg)](https://github.com/ER-brocode/swarm-nav-mesh/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![ROS2 Humble/Iron/Rolling](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron-orange.svg)](https://docs.ros.org/)

A distributed, decentralized peer-to-peer navigation mesh and multi-agent flocking telemetry engine designed for autonomous UAV drone swarms in GPS-denied and ad-hoc communication environments.

```
       [UAV-01]  <-- 5.8 GHz UDP Heartbeat -->  [UAV-02]
          \                                      /
           \                                    /
            v                                  v
     [Separation] <--- [Reynolds 3D] ---> [Cohesion]
            |                                  |
            +------------ [UAV-03] ------------+
                            |
                   [3D Geofence Enforcer]
```

---

## ⚡ Key Architectural Capabilities

### 1. 3D Reynolds Flocking & Potential Field Collision Avoidance (`src/swarm/`)
- **Separation**: Inverse-distance weighted repulsive steering prevents mid-air inter-agent collisions.
- **Alignment**: Vector consensus aligns the headings of local flockmates within a configurable sensing horizon.
- **Cohesion**: Center-of-mass attraction guarantees cluster integrity during high-speed migrations.
- **Target Seeking**: Smooth 3D pursuit of navigational waypoints and formation targets.
- **Obstacle Avoidance**: Artificial Potential Field (APF) repulsive velocity barriers against static and dynamic aerial obstacles.

### 2. Peer-to-Peer Ad-Hoc UDP Mesh Consensus (`src/mesh/`)
- Low-latency, connectionless peer-to-peer telemetry heartbeat broadcasting.
- Dynamic ad-hoc mesh adjacency tracking and network graph diameter computation.
- Automatic node liveness monitoring with dead-peer eviction.
- Raft-inspired decentralized leader election for fleet-wide mission commands.

### 3. 3D Waypoint Trajectory & Geofencing Enforcement (`src/telemetry/`)
- Minimum-jerk quintic polynomial interpolation generating smooth $C^2$ continuous flight paths.
- Real-time containment clamping within inclusion flight corridors (`INCLUSION_AABB`).
- Radial clearance repulsion from no-fly zones (`EXCLUSION_CYLINDER`).
- Automatic flight duration estimation and failsafe return-to-home boundaries.

---

## 📂 Repository File Tree

```
swarm-nav-mesh/
├── .github/
│   └── workflows/
│       └── ci.yml
├── src/
│   ├── mesh/
│   │   ├── __init__.py
│   │   └── peer.py
│   ├── swarm/
│   │   ├── __init__.py
│   │   ├── flocking.py
│   │   └── ros2_node.py
│   └── telemetry/
│       ├── __init__.py
│       └── planner.py
├── tests/
│   ├── test_mesh.py
│   ├── test_swarm.py
│   └── test_telemetry.py
├── pyproject.toml
├── setup.py
└── README.md
```

---

## 🚀 Quick Start

### Installation

```bash
git clone https://github.com/ER-brocode/swarm-nav-mesh.git
cd swarm-nav-mesh
pip install -e ".[dev]"
```

### Running Autonomous Flocking Simulator

```bash
python -m swarm.flocking
```

### Starting Decentralized Mesh Peer Nodes

```bash
# Terminal 1: Drone Alpha
python -m mesh.peer
```

### Running Full Automated Test Suite

```bash
pytest tests/ -v
```

---

## 📊 Verification & Test Metrics

- **Swarm Convergence**: Validated 6-agent flocking convergence to shared targets with zero collision breaches.
- **Mesh Consensus**: 100% loopback UDP heartbeat packet reception and dynamic leader election.
- **Geofence Safety**: Strict boundary clamping validated for both 3D bounding boxes and radial exclusion cylinders.

---

## 🛡️ License
Licensed under the [Apache 2.0 License](LICENSE). Author: **koushik4475** (<koushik4475@gmail.com>).
