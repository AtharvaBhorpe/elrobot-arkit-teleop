"""Web backend API tests — offline, stub joint data, ROS domain 77."""

import os

os.environ.setdefault("ROS_DOMAIN_ID", "77")  # NEVER touch a live session

import time

from fastapi.testclient import TestClient

from elrobot.web.server import create_app


class FakeBridge:
    """Stands in for the rclpy WebBridge in API tests (no DDS at all)."""

    def __init__(self):
        self.latest_q = {"rev_motor_01": 0.1, "rev_motor_08": 0.5}
        self.latest_stamp = time.monotonic()
        self.control_on = False
        self.published = []          # (dict) commands captured

    def commanders(self):
        return 0

    def publish_command(self, positions: dict):
        self.published.append(dict(positions))


def test_status_reports_joints_and_flags():
    app = create_app(FakeBridge())
    c = TestClient(app)
    r = c.get("/api/status")
    assert r.status_code == 200
    body = r.json()
    assert body["control_on"] is False
    assert body["commanders"] == 0
    assert body["joints"]["rev_motor_01"] == 0.1
    assert body["age_s"] < 1.0
    assert body["driver_alive"] is False   # no /joint_states publisher on 77


def test_control_toggle_and_seed():
    b = FakeBridge()
    c = TestClient(create_app(b))
    r = c.post("/api/control", json={"on": True})
    assert r.json()["control_on"] is True
    assert r.json()["seed"]["rev_motor_01"] == 0.1   # seeds from current pose
    assert b.control_on is True
    c.post("/api/control", json={"on": False})
    assert b.control_on is False


def test_ws_streams_state_and_gates_commands():
    b = FakeBridge()
    c = TestClient(create_app(b))
    with c.websocket_connect("/ws") as ws:
        first = ws.receive_json()
        assert first["type"] == "state" and "rev_motor_01" in first["joints"]
        # command while control OFF -> dropped, not published
        ws.send_json({"type": "cmd", "positions": {"rev_motor_01": 0.5}})
        ws.receive_json()                      # let the loop cycle
        assert b.published == []
        # flip on, command flows
        b.control_on = True
        ws.send_json({"type": "cmd", "positions": {"rev_motor_01": 0.5}})
        deadline = time.monotonic() + 2.0
        while not b.published and time.monotonic() < deadline:
            ws.receive_json()
        assert b.published and b.published[-1]["rev_motor_01"] == 0.5


if __name__ == "__main__":
    test_status_reports_joints_and_flags()
    test_control_toggle_and_seed()
    test_ws_streams_state_and_gates_commands()
    print("WEB API TESTS PASSED")
