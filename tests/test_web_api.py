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


if __name__ == "__main__":
    test_status_reports_joints_and_flags()
    print("WEB API TESTS PASSED")
