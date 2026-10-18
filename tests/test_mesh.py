"""Test suite for Mesh UDP peer communication, topology graph, and consensus."""

import time
import pytest

from mesh.peer import (
    ConsensusState,
    HeartbeatPacket,
    MeshNode,
    MeshTopology,
)


class TestMeshNetwork:
    """Tests peer-to-peer heartbeat, shortest-path mesh routing, and consensus election."""

    def test_heartbeat_packet_serialization(self):
        packet = HeartbeatPacket(
            sender_id="uav-01",
            term=2,
            state="LEADER",
            position=(12.5, -4.2, 30.0),
            velocity=(1.0, 0.5, -0.2),
            battery_pct=94.5,
            neighbors=["uav-02", "uav-03"],
            sequence_num=42,
            command="HOVER",
        )
        serialized = packet.serialize()
        assert isinstance(serialized, bytes)

        restored = HeartbeatPacket.deserialize(serialized)
        assert restored.sender_id == "uav-01"
        assert restored.term == 2
        assert restored.position == (12.5, -4.2, 30.0)
        assert restored.neighbors == ["uav-02", "uav-03"]
        assert restored.command == "HOVER"

    def test_mesh_topology_routing_and_diameter(self):
        # Build 4-node line graph: A -- B -- C -- D
        topo = MeshTopology("nodeA")
        topo.update_peer_links("nodeA", ["nodeB"])
        topo.update_peer_links("nodeB", ["nodeA", "nodeC"])
        topo.update_peer_links("nodeC", ["nodeB", "nodeD"])
        topo.update_peer_links("nodeD", ["nodeC"])

        assert topo.is_fully_connected() is True
        path = topo.shortest_path("nodeA", "nodeD")
        assert path == ["nodeA", "nodeB", "nodeC", "nodeD"]
        assert topo.get_network_diameter() == 3

    def test_udp_heartbeat_exchange_between_peers(self):
        node1 = MeshNode("peer-1", bind_port=0, heartbeat_interval=0.1)
        node2 = MeshNode("peer-2", bind_port=0, heartbeat_interval=0.1)

        node1.start()
        node2.start()

        # Connect endpoints
        node1.add_peer_endpoint("127.0.0.1", node2.bind_port)
        node2.add_peer_endpoint("127.0.0.1", node1.bind_port)

        try:
            # Allow heartbeats to transmit
            time.sleep(0.6)

            assert "peer-2" in node1.peers
            assert "peer-1" in node2.peers
            assert node1.peers["peer-2"].is_alive
            assert node2.peers["peer-1"].is_alive
        finally:
            node1.stop()
            node2.stop()
