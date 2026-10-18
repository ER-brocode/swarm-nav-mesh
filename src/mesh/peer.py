"""Peer-to-Peer UDP Heartbeat, Ad-Hoc Mesh Topology, and Distributed Consensus.

Features:
- UDP broadcast/unicast telemetry heartbeat packets
- Peer discovery, liveness tracking, and automatic dead-peer eviction
- Ad-hoc mesh adjacency matrix & graph diameter computation
- Raft-inspired decentralized leader election & formation command consensus
"""

from __future__ import annotations

import json
import logging
import socket
import struct
import threading
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Set, Tuple

logger = logging.getLogger("SwarmMesh")


class ConsensusState(str, Enum):
    FOLLOWER = "FOLLOWER"
    CANDIDATE = "CANDIDATE"
    LEADER = "LEADER"


@dataclass
class HeartbeatPacket:
    """Telemetry heartbeat packet exchanged across the peer-to-peer drone mesh."""
    sender_id: str
    term: int
    state: str
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    battery_pct: float
    neighbors: List[str]
    sequence_num: int
    timestamp: float = field(default_factory=time.time)
    command: Optional[str] = None

    def serialize(self) -> bytes:
        return json.dumps(asdict(self)).encode("utf-8")

    @classmethod
    def deserialize(cls, data: bytes) -> HeartbeatPacket:
        raw = json.loads(data.decode("utf-8"))
        return cls(
            sender_id=raw["sender_id"],
            term=raw["term"],
            state=raw["state"],
            position=tuple(raw["position"]),  # type: ignore
            velocity=tuple(raw["velocity"]),  # type: ignore
            battery_pct=float(raw["battery_pct"]),
            neighbors=list(raw.get("neighbors", [])),
            sequence_num=int(raw["sequence_num"]),
            timestamp=float(raw.get("timestamp", time.time())),
            command=raw.get("command"),
        )


@dataclass
class PeerState:
    """State record maintained for every detected peer in the mesh."""
    node_id: str
    address: Tuple[str, int]
    last_seen: float
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    battery_pct: float
    term: int
    state: str
    neighbors: List[str]
    latency_ms: float = 0.0

    @property
    def is_alive(self) -> bool:
        # 3.0 seconds heartbeat timeout
        return (time.time() - self.last_seen) < 3.0


class MeshTopology:
    """Calculates graph connectivity, routing hops, and mesh health metrics."""

    def __init__(self, local_id: str):
        self.local_id = local_id
        self.adjacency: Dict[str, Set[str]] = {local_id: set()}

    def update_peer_links(self, node_id: str, neighbors: List[str]) -> None:
        if node_id not in self.adjacency:
            self.adjacency[node_id] = set()
        self.adjacency[node_id] = set(neighbors)
        # Ensure undirected edge representation
        for n in neighbors:
            if n not in self.adjacency:
                self.adjacency[n] = set()
            self.adjacency[n].add(node_id)

    def remove_peer(self, node_id: str) -> None:
        if node_id in self.adjacency:
            del self.adjacency[node_id]
        for peers in self.adjacency.values():
            peers.discard(node_id)

    def shortest_path(self, start: str, end: str) -> Optional[List[str]]:
        """Breadth-First Search for minimum-hop routing through the mesh."""
        if start not in self.adjacency or end not in self.adjacency:
            return None
        if start == end:
            return [start]

        queue: List[Tuple[str, List[str]]] = [(start, [start])]
        visited: Set[str] = {start}

        while queue:
            curr, path = queue.pop(0)
            for nbr in self.adjacency.get(curr, set()):
                if nbr == end:
                    return path + [end]
                if nbr not in visited:
                    visited.add(nbr)
                    queue.append((nbr, path + [nbr]))
        return None

    def get_network_diameter(self) -> int:
        """Maximum shortest-path distance between any pair of connected nodes."""
        nodes = list(self.adjacency.keys())
        if len(nodes) <= 1:
            return 0
        max_dist = 0
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                path = self.shortest_path(nodes[i], nodes[j])
                if path is not None:
                    dist = len(path) - 1
                    if dist > max_dist:
                        max_dist = dist
        return max_dist

    def is_fully_connected(self) -> bool:
        nodes = list(self.adjacency.keys())
        if not nodes:
            return True
        visited = set()
        queue = [nodes[0]]
        visited.add(nodes[0])

        while queue:
            curr = queue.pop(0)
            for nbr in self.adjacency.get(curr, set()):
                if nbr not in visited:
                    visited.add(nbr)
                    queue.append(nbr)
        return len(visited) == len(nodes)


class MeshNode:
    """Fully decentralized peer-to-peer mesh node with consensus & telemetry routing."""

    def __init__(
        self,
        node_id: str,
        bind_host: str = "127.0.0.1",
        bind_port: int = 0,
        broadcast_port: int = 19447,
        heartbeat_interval: float = 0.5,
    ):
        self.node_id = node_id
        self.bind_host = bind_host
        self.bind_port = bind_port
        self.broadcast_port = broadcast_port
        self.heartbeat_interval = heartbeat_interval

        # Consensus state
        self.term: int = 0
        self.consensus_state: ConsensusState = ConsensusState.FOLLOWER
        self.leader_id: Optional[str] = None
        self.voted_for: Optional[str] = None
        self.votes_received: Set[str] = set()

        # Telemetry state
        self.position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.battery_pct: float = 100.0
        self.sequence_num: int = 0
        self.latest_command: Optional[str] = None

        # Network state
        self.peers: Dict[str, PeerState] = {}
        self.topology = MeshTopology(self.node_id)
        self.known_endpoints: Set[Tuple[str, int]] = set()

        # Socket setup
        self._sock: Optional[socket.socket] = None
        self._running = False
        self._threads: List[threading.Thread] = []
        self._lock = threading.RLock()
        self._on_command_callback: Optional[Callable[[str], None]] = None

    def start(self) -> None:
        """Bind socket and start background listening and heartbeat dispatching."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.bind((self.bind_host, self.bind_port))
        # Resolve dynamic port if 0 was given
        self.bind_port = self._sock.getsockname()[1]
        self._running = True

        recv_thread = threading.Thread(target=self._recv_loop, name=f"mesh-{self.node_id}-recv", daemon=True)
        heartbeat_thread = threading.Thread(target=self._heartbeat_loop, name=f"mesh-{self.node_id}-hb", daemon=True)
        consensus_thread = threading.Thread(target=self._consensus_loop, name=f"mesh-{self.node_id}-raft", daemon=True)

        self._threads = [recv_thread, heartbeat_thread, consensus_thread]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        for t in self._threads:
            t.join(timeout=1.0)

    def add_peer_endpoint(self, host: str, port: int) -> None:
        with self._lock:
            self.known_endpoints.add((host, port))

    def update_telemetry(self, pos: Tuple[float, float, float], vel: Tuple[float, float, float], battery: float) -> None:
        with self._lock:
            self.position = pos
            self.velocity = vel
            self.battery_pct = battery

    def broadcast_command(self, cmd: str) -> None:
        """Leader broadcasts command across mesh."""
        with self._lock:
            self.latest_command = cmd
            if self._on_command_callback:
                self._on_command_callback(cmd)

    def set_command_callback(self, cb: Callable[[str], None]) -> None:
        self._on_command_callback = cb

    def _recv_loop(self) -> None:
        while self._running:
            try:
                if not self._sock:
                    break
                data, addr = self._sock.recvfrom(65535)
                packet = HeartbeatPacket.deserialize(data)
                self._handle_packet(packet, addr)
            except (socket.timeout, OSError):
                continue
            except Exception as e:
                if self._running:
                    logger.debug(f"Receive loop error: {e}")

    def _handle_packet(self, packet: HeartbeatPacket, addr: Tuple[str, int]) -> None:
        if packet.sender_id == self.node_id:
            return

        now = time.time()
        with self._lock:
            self.known_endpoints.add(addr)
            latency = (now - packet.timestamp) * 1000.0

            # Update peer table
            self.peers[packet.sender_id] = PeerState(
                node_id=packet.sender_id,
                address=addr,
                last_seen=now,
                position=packet.position,
                velocity=packet.velocity,
                battery_pct=packet.battery_pct,
                term=packet.term,
                state=packet.state,
                neighbors=packet.neighbors,
                latency_ms=max(0.0, latency),
            )

            # Update mesh graph topology
            self.topology.update_peer_links(packet.sender_id, packet.neighbors)
            if self.node_id not in self.topology.adjacency[self.node_id]:
                self.topology.adjacency[self.node_id].add(packet.sender_id)

            # Consensus logic
            if packet.term > self.term:
                self.term = packet.term
                self.consensus_state = ConsensusState.FOLLOWER
                self.voted_for = None
                if packet.state == ConsensusState.LEADER:
                    self.leader_id = packet.sender_id

            if packet.state == ConsensusState.LEADER and packet.term == self.term:
                self.leader_id = packet.sender_id

            # Dispatch newly received swarm command
            if packet.command and packet.command != self.latest_command:
                self.latest_command = packet.command
                if self._on_command_callback:
                    self._on_command_callback(packet.command)

    def _heartbeat_loop(self) -> None:
        while self._running:
            with self._lock:
                self.sequence_num += 1
                active_neighbors = [pid for pid, p in self.peers.items() if p.is_alive]
                packet = HeartbeatPacket(
                    sender_id=self.node_id,
                    term=self.term,
                    state=self.consensus_state.value,
                    position=self.position,
                    velocity=self.velocity,
                    battery_pct=self.battery_pct,
                    neighbors=active_neighbors,
                    sequence_num=self.sequence_num,
                    command=self.latest_command,
                )
                payload = packet.serialize()
                endpoints = list(self.known_endpoints)

            # Transmit to known peers and broadcast port
            if self._sock and not self._sock._closed:  # type: ignore
                try:
                    self._sock.sendto(payload, ("127.0.0.1", self.broadcast_port))
                except Exception:
                    pass
                for ep in endpoints:
                    try:
                        self._sock.sendto(payload, ep)
                    except Exception:
                        pass

            time.sleep(self.heartbeat_interval)

    def _consensus_loop(self) -> None:
        """Periodic Raft consensus election check."""
        while self._running:
            time.sleep(1.0)
            with self._lock:
                # Evict timed-out peers
                now = time.time()
                dead = [pid for pid, p in self.peers.items() if (now - p.last_seen) > 3.0]
                for pid in dead:
                    del self.peers[pid]
                    self.topology.remove_peer(pid)
                    if self.leader_id == pid:
                        self.leader_id = None

                # If leader missing, promote candidate and conduct election
                alive_peers = [p for p in self.peers.values() if p.is_alive]
                total_cluster = len(alive_peers) + 1
                quorum = (total_cluster // 2) + 1

                if self.leader_id is None and self.consensus_state != ConsensusState.LEADER:
                    # Deterministic priority or node comparison
                    self.consensus_state = ConsensusState.CANDIDATE
                    self.term += 1
                    self.voted_for = self.node_id
                    self.votes_received = {self.node_id}

                    # If majority quorum reached (e.g. single node or highest ID)
                    if len(self.votes_received) >= quorum or all(self.node_id > p.node_id for p in alive_peers):
                        self.consensus_state = ConsensusState.LEADER
                        self.leader_id = self.node_id

    def get_active_peer_count(self) -> int:
        with self._lock:
            return sum(1 for p in self.peers.values() if p.is_alive)


def main():
    """CLI demonstration of two peer nodes discovering each other via loopback."""
    print("=== Swarm Nav Mesh - UDP Mesh Consensus Node ===")
    node1 = MeshNode("drone-alpha", bind_port=19401)
    node2 = MeshNode("drone-beta", bind_port=19402)

    node1.add_peer_endpoint("127.0.0.1", 19402)
    node2.add_peer_endpoint("127.0.0.1", 19401)

    node1.start()
    node2.start()

    try:
        print("Nodes started, exchanging heartbeats for 3 seconds...")
        time.sleep(3.0)
        print(f"Node 1 active peers: {node1.get_active_peer_count()}, leader: {node1.leader_id}")
        print(f"Node 2 active peers: {node2.get_active_peer_count()}, leader: {node2.leader_id}")
    finally:
        node1.stop()
        node2.stop()
        print("Mesh test complete.")


if __name__ == "__main__":
    main()
