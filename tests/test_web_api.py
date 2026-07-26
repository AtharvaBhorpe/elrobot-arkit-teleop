"""Web backend API tests — offline, stub joint data, ROS domain 77."""

import os

os.environ.setdefault("ROS_DOMAIN_ID", "77")  # NEVER touch a live session

import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # for `stubs` below

from fastapi.testclient import TestClient

from elrobot.web.server import create_app


class FakeBridge:
    """Stands in for the rclpy WebBridge in API tests (no DDS at all)."""

    def __init__(self):
        self.latest_q = {"rev_motor_01": 0.1, "rev_motor_08": 0.5}
        self.latest_stamp = time.monotonic()
        self.control_on = False
        self.published = []          # (dict) commands captured
        self.record_cmds = []        # str commands captured

    def commanders(self):
        return 0

    def publish_command(self, positions: dict):
        self.published.append(dict(positions))

    def publish_record_cmd(self, cmd: str):
        self.record_cmds.append(cmd)


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


def test_mjpeg_placeholder_when_no_camera():
    # The /cam/{name} generator streams forever (real browsers consume it
    # frame by frame); TestClient's ASGI transport buffers a response
    # fully before returning it, so reading a live HTTP response here
    # would hang forever waiting for a stream that never ends. Instead,
    # call the route's endpoint function directly and pull one frame
    # from its real body_iterator - same production code, no HTTP layer.
    import asyncio

    b = FakeBridge()
    b.latest_jpeg = {}                     # no camera has published
    app = create_app(b)
    route = next(r for r in app.routes if getattr(r, "path", None) == "/cam/{name}")

    async def first_frame(name):
        resp = route.endpoint(name=name)
        it = resp.body_iterator
        chunk = await it.__anext__()
        await it.aclose()
        return resp, chunk

    resp, chunk = asyncio.run(first_frame("wrist"))
    assert resp.status_code == 200
    assert "multipart/x-mixed-replace" in resp.media_type
    assert b"--frame" in chunk and b"\xff\xd8" in chunk   # JPEG SOI

    c = TestClient(app)
    assert c.get("/cam/nope").status_code == 404


def test_static_urdf_and_meshes_served():
    b = FakeBridge()
    c = TestClient(create_app(b))
    assert c.get("/").status_code == 200
    r = c.get("/urdf")
    assert r.status_code == 200
    # relative "meshes/x.dae", NOT "/meshes/x.dae" - URDFLoader resolves
    # non-package:// mesh paths via plain string concatenation with its
    # workingPath ("/" for a URDF served at /urdf), so a leading slash here
    # would concatenate into "//meshes/x.dae", a protocol-relative URL the
    # browser reads as host "meshes" (confirmed against the real vendored
    # URDFLoader.js - see server.py's comment on the /urdf route)
    assert 'filename="meshes/' in r.text
    assert 'filename="/meshes/' not in r.text
    assert "data/viz_meshes" not in r.text          # no filesystem paths leak
    one = r.text.split('filename="meshes/')[1].split('"')[0]
    assert c.get(f"/meshes/{one}").status_code == 200


def test_calib_refuses_while_driver_alive():
    b = FakeBridge()
    b.driver_alive = lambda: True
    c = TestClient(create_app(b, bus_factory=lambda: None))
    assert c.post("/api/calib/start").status_code == 409


def test_calib_eeprom_needs_typed_confirmation():
    b = FakeBridge()
    b.driver_alive = lambda: False
    from stubs import StubBus

    # all 8 motors present - the "preflight" sweep phase reads all of them
    # at once via sync_read; a bus seeded with only one motor would crash
    # the background sweep thread (silently, since it's a daemon thread)
    positions = {f"rev_motor_{i:02d}": 2000 for i in range(1, 9)}
    c = TestClient(create_app(b, bus_factory=lambda: StubBus(positions)))
    assert c.post("/api/calib/start").json()["state"] == "preflight"
    c.post("/api/calib/sweep/begin")
    c.post("/api/calib/sweep/end")
    r = c.post("/api/calib/eeprom", json={"confirm": "erase"})
    assert r.status_code == 400                       # exact string required
    r = c.post("/api/calib/eeprom", json={"confirm": "ERASE"})
    assert r.status_code == 200


def test_calib_full_flow_writes_table():
    b = FakeBridge()
    b.driver_alive = lambda: False
    from stubs import StubBus

    positions = {f"rev_motor_{i:02d}": 2000 for i in range(1, 9)}
    bus = StubBus(positions)
    c = TestClient(create_app(b, bus_factory=lambda: bus))
    c.post("/api/calib/start")
    c.post("/api/calib/sweep/begin")
    c.post("/api/calib/sweep/end")                    # -> gate
    c.post("/api/calib/eeprom", json={"confirm": "ERASE"})   # -> eeprom_done

    # M1b phase A: sweep each full-turn joint (05 then 07)
    c.post("/api/calib/sweep/begin")
    c.post("/api/calib/sweep/end")
    r = c.post("/api/calib/sweep/begin")
    assert r.json()["state"] == "fullturn"
    r = c.post("/api/calib/sweep/end")
    assert r.json()["state"] == "signs"                # both full-turn joints done

    for i in range(1, 8):
        c.post("/api/calib/sign", json={"joint": f"rev_motor_{i:02d}", "flip": False})

    # NEVER the real calibration/urdf_ticks.json - see server.py's comment
    # on the finish route. A prior version of this test wrote fake offsets
    # into the real, hand-measured file before this override existed.
    scratch = tempfile.mktemp(suffix=".json")
    try:
        r = c.post("/api/calib/finish", json={"out": scratch})
        assert r.status_code == 200
        body = r.json()
        assert body["state"] == "done"
        assert set(body["table"]) == {f"rev_motor_{i:02d}" for i in range(1, 9)}
        assert body["fk"]["height_m"] is not None
        assert bus.connected is False                  # disconnected on finish
        assert Path(scratch).exists()                  # wrote to the override, not the real file
    finally:
        Path(scratch).unlink(missing_ok=True)


def test_record_relays_valid_commands_only():
    b = FakeBridge()
    c = TestClient(create_app(b))
    r = c.post("/api/record", json={"cmd": "start"})
    assert r.status_code == 200 and b.record_cmds == ["start"]
    r = c.post("/api/record", json={"cmd": "bogus"})
    assert r.status_code == 400
    assert b.record_cmds == ["start"]                  # bogus cmd never relayed


if __name__ == "__main__":
    test_status_reports_joints_and_flags()
    test_control_toggle_and_seed()
    test_ws_streams_state_and_gates_commands()
    test_mjpeg_placeholder_when_no_camera()
    test_static_urdf_and_meshes_served()
    test_calib_refuses_while_driver_alive()
    test_calib_eeprom_needs_typed_confirmation()
    test_calib_full_flow_writes_table()
    test_record_relays_valid_commands_only()
    print("WEB API TESTS PASSED")
