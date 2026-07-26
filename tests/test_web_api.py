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
        self.record_status = None
        self.record_stamp = 0.0

    def commanders(self):
        return 0

    def publish_command(self, positions: dict):
        self.published.append(dict(positions))

    def publish_record_cmd(self, cmd: str):
        self.record_cmds.append(cmd)

    def record_status_fresh(self):
        # Must mirror WebBridge: the WS sender calls this every tick, and a
        # missing method raises inside that task, killing the state stream
        # with no error visible to the client - the socket just goes quiet.
        import elrobot.web.server as srv
        return srv.WebBridge.record_status_fresh(self)


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


def test_cam_frame_endpoint_returns_single_jpeg():
    # The cockpit UI polls this instead of relying on multipart/x-mixed-
    # replace inside <img>, which recent Chromium versions don't reliably
    # paint (confirmed live: correct headers/framing, data genuinely
    # transferring, image never visually updating).
    b = FakeBridge()
    b.latest_jpeg = {}
    c = TestClient(create_app(b))
    r = c.get("/cam/wrist/frame")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/jpeg"
    assert r.headers["cache-control"] == "no-store"
    assert r.content[:2] == b"\xff\xd8"       # JPEG SOI, not multipart-wrapped
    assert c.get("/cam/nope/frame").status_code == 404


def test_static_urdf_and_meshes_served():
    b = FakeBridge()
    c = TestClient(create_app(b))
    index_resp = c.get("/")
    assert index_resp.status_code == 200
    r = c.get("/urdf")
    assert r.status_code == 200
    # no-store on both: a stale cached /urdf once reproduced an
    # already-fixed mesh-path bug in a live browser until a hard refresh
    assert index_resp.headers["cache-control"] == "no-store"
    assert r.headers["cache-control"] == "no-store"
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


def _calib_client(positions=None):
    b = FakeBridge()
    b.driver_alive = lambda: False
    from stubs import StubBus

    # all 8 motors present - the sweep phase reads them together via
    # sync_read; a partially-seeded bus is the failure case tested below
    positions = positions or {f"rev_motor_{i:02d}": 2000 for i in range(1, 9)}
    bus = StubBus(positions)
    return TestClient(create_app(b, bus_factory=lambda: bus)), bus


def test_calib_eeprom_needs_typed_confirmation():
    c, _ = _calib_client()
    assert c.post("/api/calib/start").json()["state"] == "preflight"
    # EEPROM comes FIRST, straight from preflight - before any sweep
    r = c.post("/api/calib/eeprom", json={"confirm": "erase"})
    assert r.status_code == 400                       # exact string required
    r = c.post("/api/calib/eeprom", json={"confirm": "ERASE"})
    assert r.status_code == 200 and r.json()["state"] == "homed"


def test_calib_writes_eeprom_before_sweeping():
    """Order is load-bearing: set_half_turn_homings() redefines what
    Present_Position returns, so ranges recorded BEFORE it are in a frame the
    write invalidates. m1a_calibrate writes homing, THEN sweeps; an earlier
    version of this wizard swept first and would have derived offsets from
    stale ranges - a table that makes the real arm move wrong."""
    c, bus = _calib_client()
    c.post("/api/calib/start")
    # sweeping is not reachable until the homing write has happened
    assert c.post("/api/calib/sweep/begin").status_code == 409
    assert bus.set_half_turn_homings_calls == 0
    c.post("/api/calib/eeprom", json={"confirm": "ERASE"})
    assert bus.set_half_turn_homings_calls == 1
    assert c.post("/api/calib/sweep/begin").json()["state"] == "sweeping"


def test_calib_surfaces_sweep_failure_instead_of_advancing():
    """A silent motor made read_ranges raise inside a daemon thread, whose
    exception was swallowed: ranges stayed empty, gate_ranges({}) returned an
    empty list that rendered as blank (looking like a pass), the state still
    advanced, and the destructive write sat right after it."""
    positions = {f"rev_motor_{i:02d}": 2000 for i in range(1, 9)}
    del positions["rev_motor_04"]                    # motor 4 never answers
    c, _ = _calib_client(positions)
    c.post("/api/calib/start")
    c.post("/api/calib/eeprom", json={"confirm": "ERASE"})
    c.post("/api/calib/sweep/begin")
    time.sleep(0.3)
    r = c.post("/api/calib/sweep/end")
    assert r.status_code == 500                      # loud, not silent
    state = c.get("/api/calib/state").json()
    assert state["error"] and "rev_motor_04" in state["error"]
    assert state["state"] != "gate"                  # did NOT advance


def test_calib_finish_refuses_partial_table():
    """urdf_ticks.json is hand-measured physical truth; a missing joint would
    silently become a wrong offset."""
    c, _ = _calib_client()
    c.post("/api/calib/start")
    c.post("/api/calib/eeprom", json={"confirm": "ERASE"})
    c.post("/api/calib/sweep/begin")
    time.sleep(0.15)
    c.post("/api/calib/sweep/end")                   # -> gate (05/07 missing)
    # jump straight at finish without the full-turn sweeps
    c.post("/api/calib/sign", json={"joint": "rev_motor_01", "flip": False})
    r = c.post("/api/calib/finish", json={"out": "/tmp/should-not-exist.json"})
    assert r.status_code == 409
    assert not Path("/tmp/should-not-exist.json").exists()


def test_calib_start_disconnects_bus_if_setup_fails_partway():
    """connect() succeeding and disable_torque() then throwing used to drop a
    live connected bus with no reference left to close it - the port stayed
    held and Start could open a SECOND connection to the same UART."""
    from stubs import StubBus

    b = FakeBridge()
    b.driver_alive = lambda: False
    bus = StubBus({f"rev_motor_{i:02d}": 2000 for i in range(1, 9)})
    bus.disable_torque = lambda: (_ for _ in ()).throw(OSError("bus glitch"))
    c = TestClient(create_app(b, bus_factory=lambda: bus))
    r = c.post("/api/calib/start")
    assert r.status_code == 409
    assert bus.connected is False          # released, not leaked


def test_calib_finish_does_not_write_table_when_final_read_fails():
    """save_table used to run BEFORE the final pose read. A throw there left
    calibration/urdf_ticks.json already overwritten while the UI reported a
    failure and re-enabled Finish - the operator believing nothing happened."""
    c, bus = _calib_client()
    c.post("/api/calib/start")
    c.post("/api/calib/eeprom", json={"confirm": "ERASE"})
    for _ in range(3):                     # arm sweep + both full-turn joints
        c.post("/api/calib/sweep/begin")
        time.sleep(0.15)
        c.post("/api/calib/sweep/end")
    # Installed only now, after the sweeps: the very next sync_read is
    # finish()'s final pose read, which is the one that must not be able to
    # leave a written table behind.
    def dead_bus(item, names=None, normalize=False):
        raise OSError("serial timeout")

    bus.sync_read = dead_bus
    scratch = tempfile.mktemp(suffix=".json")
    r = c.post("/api/calib/finish", json={"out": scratch})
    assert r.status_code == 500
    assert not Path(scratch).exists()      # table NOT written
    assert "NOT written" in c.get("/api/calib/state").json()["error"]


def test_calib_abort_releases_the_serial_port():
    c, bus = _calib_client()
    c.post("/api/calib/start")
    assert bus.connected is True
    r = c.post("/api/calib/abort")
    assert r.status_code == 200 and r.json()["state"] == "idle"
    assert bus.connected is False          # driver can have the port back


def test_record_status_goes_stale_when_recorder_dies():
    """Caching the last /record/status forever meant killing the recorder
    mid-episode left the cockpit showing "recording" with Stop enabled."""
    import elrobot.web.server as srv

    b = FakeBridge()
    b.record_status = {"recording": True, "episodes": 0, "frames": 42}
    b.record_stamp = time.monotonic()
    assert b.record_status_fresh() == b.record_status      # fresh
    b.record_stamp = time.monotonic() - (srv.RECORD_STALE_S + 1)
    assert b.record_status_fresh() is None                 # expired


def test_only_first_ws_client_may_command():
    """control_on is a single shared flag, so two tabs would each run their
    own 25 Hz publisher against /joint_command with no arbitration - and the
    two-commander banner cannot see them (it filters by ROS node name and
    both tabs share this one node).

    Driven through app.state rather than two real websockets: TestClient runs
    the ASGI app on one blocking portal and cannot hold two concurrent
    websocket sessions (attempting it deadlocks), so the arbitration rule is
    exercised directly. The single-client path through the real transport is
    covered by test_ws_streams_state_and_gates_commands."""
    b = FakeBridge()
    b.control_on = True
    app = create_app(b)
    tab_a, tab_b = object(), object()          # stand-ins for two sockets

    app.state.client_joined(tab_a)
    app.state.client_joined(tab_b)
    assert app.state.may_command(tab_a) is True     # first in owns control
    assert app.state.may_command(tab_b) is False    # second is monitor-only

    # owner leaves -> the remaining tab is promoted, not left locked out
    app.state.client_gone(tab_a)
    assert app.state.may_command(tab_b) is True

    # control off -> nobody commands, even the owner
    b.control_on = False
    assert app.state.may_command(tab_b) is False


def test_control_resets_when_last_client_disconnects():
    """The client's unload POST races page teardown; losing that race used to
    leave control_on stuck True with no publisher, so a later tab found the
    server already 'in control'."""
    b = FakeBridge()
    app = create_app(b)
    tab = object()
    app.state.client_joined(tab)
    b.control_on = True
    app.state.client_gone(tab)
    assert b.control_on is False       # authoritative server-side reset


def test_calib_start_reports_port_open_failure_cleanly():
    b = FakeBridge()
    b.driver_alive = lambda: False

    def boom():
        raise OSError("[Errno 16] Device or resource busy: '/dev/ttyACM0'")

    c = TestClient(create_app(b, bus_factory=boom))
    r = c.post("/api/calib/start")
    assert r.status_code == 409                      # not an opaque 500
    assert "busy" in r.json()["detail"]


def test_calib_full_flow_writes_table():
    c, bus = _calib_client()
    c.post("/api/calib/start")
    # M1a order: park -> EEPROM homing write -> THEN sweep the ranges
    c.post("/api/calib/eeprom", json={"confirm": "ERASE"})   # -> homed
    c.post("/api/calib/sweep/begin")
    time.sleep(0.15)
    c.post("/api/calib/sweep/end")                    # -> gate

    # M1b phase A: sweep each full-turn joint (05 then 07)
    r = c.post("/api/calib/sweep/begin")
    assert r.json()["state"] == "fullturn"
    time.sleep(0.15)
    c.post("/api/calib/sweep/end")
    r = c.post("/api/calib/sweep/begin")
    assert r.json()["state"] == "fullturn"
    time.sleep(0.15)
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
    test_cam_frame_endpoint_returns_single_jpeg()
    test_static_urdf_and_meshes_served()
    test_calib_refuses_while_driver_alive()
    test_calib_eeprom_needs_typed_confirmation()
    test_calib_writes_eeprom_before_sweeping()
    test_calib_surfaces_sweep_failure_instead_of_advancing()
    test_calib_finish_refuses_partial_table()
    test_calib_start_disconnects_bus_if_setup_fails_partway()
    test_calib_finish_does_not_write_table_when_final_read_fails()
    test_calib_abort_releases_the_serial_port()
    test_record_status_goes_stale_when_recorder_dies()
    test_only_first_ws_client_may_command()
    test_control_resets_when_last_client_disconnects()
    test_calib_start_reports_port_open_failure_cleanly()
    test_calib_full_flow_writes_table()
    test_record_relays_valid_commands_only()
    print("WEB API TESTS PASSED")
