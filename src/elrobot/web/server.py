"""Web cockpit backend: FastAPI + an embedded rclpy node.

An ordinary DDS citizen — subscribes /joint_states and camera images,
publishes /joint_command only while WEB CONTROL is on. The driver keeps
sole serial ownership and ALL safety gates. LAN only, no auth: this page
commands a robot — do not port-forward it.

    pixi run web        # http://<host>:8080
"""

import asyncio
import os
import re
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from fastapi import (
    FastAPI,
    HTTPException,
    Response,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import FileResponse, HTMLResponse, PlainTextResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_JOINT
from elrobot.web.calib import CalibError, CalibSession

JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
STALE_S = 1.0

# Serial port the CALIBRATION WIZARD opens (nothing else here touches serial).
# Same PORT env knob every other task honours - this arm has already moved
# from ttyACM0 to ttyACM1 once after a replug.
DEFAULT_PORT = os.environ.get("PORT", "/dev/ttyACM0")

ROOT = Path(__file__).resolve().parents[3]      # repo root
STATIC = Path(__file__).resolve().parent / "static"


def _placeholder(label: str) -> bytes:
    img = np.zeros((360, 640, 3), dtype=np.uint8) + 18
    cv2.putText(img, f"no signal - {label}", (40, 190),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (120, 120, 120), 1)
    return cv2.imencode(".jpg", img)[1].tobytes()


class WebBridge:
    """rclpy node wrapper; constructed lazily so tests can use a fake."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image, JointState

        rclpy.init()
        self._JointState = JointState

        class _N(Node):
            pass

        from std_msgs.msg import String

        self.node = _N("web_cockpit")
        self.latest_q: dict[str, float] = {}
        self.latest_stamp = 0.0
        self.control_on = False
        self.latest_jpeg: dict[str, tuple[bytes, float]] = {}
        self.record_status: dict | None = None
        self.node.create_subscription(
            JointState, "/joint_states", self._on_js, 1)
        for name, topic in (("wrist", "/wrist_cam/image"),
                            ("ext", "/ext_cam/image")):
            self.node.create_subscription(
                Image, topic,
                lambda m, n=name: self._on_img(m, n), 1)
        self.node.create_subscription(String, "/record/status", self._on_record, 1)
        self._record_pub = self.node.create_publisher(String, "/record/cmd", 1)
        self._pub = self.node.create_publisher(JointState, "/joint_command", 1)
        self._spin = threading.Thread(
            target=rclpy.spin, args=(self.node,), daemon=True)
        self._spin.start()

    def _on_js(self, msg):
        self.latest_q = dict(zip(msg.name, msg.position))
        self.latest_stamp = time.monotonic()

    def _on_record(self, msg):
        import json
        self.record_status = json.loads(msg.data)

    def publish_record_cmd(self, cmd: str):
        from std_msgs.msg import String
        self._record_pub.publish(String(data=cmd))

    def _on_img(self, msg, name):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        ok, jpg = cv2.imencode(".jpg", frame,
                               [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.latest_jpeg[name] = (jpg.tobytes(), time.monotonic())

    def commanders(self) -> int:
        """Other /joint_command publishers on the graph (phone ik, jog...)."""
        infos = self.node.get_publishers_info_by_topic("/joint_command")
        return sum(1 for i in infos if i.node_name != "web_cockpit")

    def publish_command(self, positions: dict):
        msg = self._JointState()
        msg.name = list(positions)
        msg.position = [float(v) for v in positions.values()]
        self._pub.publish(msg)

    def driver_alive(self) -> bool:
        return len(self.node.get_publishers_info_by_topic("/joint_states")) > 0


def create_app(bridge, bus_factory=None, port=DEFAULT_PORT) -> FastAPI:
    app = FastAPI(title="elrobot cockpit")
    app.state.bridge = bridge
    calib = CalibSession(bus_factory=bus_factory, port=port)

    @app.get("/api/status")
    def status():
        alive = (bridge.driver_alive()
                 if hasattr(bridge, "driver_alive") else False)
        return {
            "driver_alive": alive,
            "control_on": bridge.control_on,
            "commanders": bridge.commanders(),
            "joints": bridge.latest_q,
            "age_s": time.monotonic() - bridge.latest_stamp,
        }

    @app.post("/api/calib/start")
    def calib_start():
        alive = bridge.driver_alive() if hasattr(bridge, "driver_alive") else False
        try:
            return calib.start(alive)
        except CalibError as e:
            raise HTTPException(e.code, e.detail) from e

    @app.post("/api/calib/sweep/begin")
    def calib_sweep_begin():
        try:
            return calib.sweep_begin()
        except CalibError as e:
            raise HTTPException(e.code, e.detail) from e

    @app.post("/api/calib/sweep/end")
    def calib_sweep_end():
        try:
            return calib.sweep_end()
        except CalibError as e:
            raise HTTPException(e.code, e.detail) from e

    @app.get("/api/calib/state")
    def calib_state():
        return calib.snapshot()

    @app.post("/api/calib/eeprom")
    def calib_eeprom(body: dict):
        try:
            return calib.eeprom(body.get("confirm", ""))
        except CalibError as e:
            raise HTTPException(e.code, e.detail) from e

    @app.post("/api/calib/sign")
    def calib_sign(body: dict):
        try:
            return calib.sign(body["joint"], bool(body.get("flip")))
        except CalibError as e:
            raise HTTPException(e.code, e.detail) from e

    @app.post("/api/calib/finish")
    def calib_finish(body: dict = None):
        # `out` override exists ONLY so offline tests never touch the real,
        # hand-measured calibration/urdf_ticks.json - the wizard UI never
        # sends it, so real use always writes the real path.
        out = (body or {}).get("out", "calibration/urdf_ticks.json")
        try:
            return calib.finish(out=out)
        except CalibError as e:
            raise HTTPException(e.code, e.detail) from e

    @app.post("/api/control")
    def control(body: dict):
        bridge.control_on = bool(body.get("on"))
        return {"control_on": bridge.control_on,
                "seed": dict(bridge.latest_q) if bridge.control_on else {}}

    @app.post("/api/record")
    def record(body: dict):
        cmd = body.get("cmd")
        if cmd not in ("start", "stop", "discard"):
            raise HTTPException(400, "cmd must be start, stop, or discard")
        bridge.publish_record_cmd(cmd)
        return {"sent": cmd}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()

        async def sender():
            while True:
                alive = (bridge.driver_alive()
                         if hasattr(bridge, "driver_alive") else False)
                await sock.send_json({
                    "type": "state", "joints": bridge.latest_q,
                    "age_s": time.monotonic() - bridge.latest_stamp,
                    "control_on": bridge.control_on,
                    "commanders": bridge.commanders(),
                    "driver_alive": alive,
                    "record": getattr(bridge, "record_status", None),
                })
                await asyncio.sleep(1 / 30)

        send_task = asyncio.create_task(sender())
        try:
            while True:
                msg = await sock.receive_json()
                if msg.get("type") == "cmd" and bridge.control_on:
                    bridge.publish_command(msg["positions"])
        except WebSocketDisconnect:
            pass
        finally:
            send_task.cancel()
            # tab gone -> stop commanding; driver deadman freezes the arm

    def _current_jpeg(name: str) -> bytes:
        jpeg, ts = getattr(bridge, "latest_jpeg", {}).get(name, (None, 0.0))
        if jpeg is None or time.monotonic() - ts > 2.0:
            jpeg = _placeholder(name)
        return jpeg

    @app.get("/cam/{name}/frame")
    def cam_frame(name: str):
        # Single-frame endpoint the cockpit UI polls (see app.js): recent
        # Chromium/Blink versions unreliably paint multipart/x-mixed-replace
        # inside <img> - confirmed live (correct headers, correct multipart
        # framing, tens of MB genuinely transferred, image never visually
        # updated). fetch() + Blob + createObjectURL works everywhere.
        if name not in ("wrist", "ext"):
            raise HTTPException(404)
        return Response(content=_current_jpeg(name), media_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})

    @app.get("/cam/{name}")
    def cam(name: str):
        # Multipart MJPEG stream - kept for external viewers (VLC, mjpeg
        # clients) that handle it fine; the cockpit UI itself uses
        # /cam/{name}/frame above instead, for the reason noted there.
        if name not in ("wrist", "ext"):
            raise HTTPException(404)

        def gen():
            # sync generator (Starlette runs it in a thread pool): a
            # blocking time.sleep here is fine and, unlike an async
            # generator's asyncio.sleep, closes cleanly on early client
            # disconnect instead of leaking a live task in the event loop
            while True:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + _current_jpeg(name) + b"\r\n")
                time.sleep(1 / 15)

        return StreamingResponse(
            gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    @app.get("/", response_class=HTMLResponse)
    def index(response: Response):
        # no-store: index.html and /urdf are read from disk on every
        # request and can change between sessions (URDF regeneration,
        # active dev). A stale cached /urdf here once masked a real fix -
        # the browser kept resolving mesh paths from a cached pre-fix
        # copy, reproducing an already-fixed bug until a hard refresh.
        response.headers["Cache-Control"] = "no-store"
        return (STATIC / "index.html").read_text()

    @app.get("/urdf", response_class=PlainTextResponse)
    def urdf(response: Response):
        response.headers["Cache-Control"] = "no-store"
        text = (ROOT / "docs" / "urdf_Elrobot_viz.urdf").read_text()
        # 'filename="...anything.../mesh.dae"' -> 'filename="meshes/mesh.dae"'
        # (relative, NOT "/meshes/...") - URDFLoader resolves non-package://
        # mesh paths as `workingPath + path` via plain string concatenation
        # (its own resolvePath()), and workingPath for a URDF served from
        # /urdf is THREE.LoaderUtils.extractUrlBase("/urdf") = "/". An
        # absolute "/meshes/x.dae" here would concatenate to "//meshes/x.dae"
        # - a PROTOCOL-RELATIVE URL the browser reads as host "meshes",
        # silently failing every mesh fetch (confirmed: robot never
        # rendered, only the grid, despite /urdf itself returning 200).
        # A relative "meshes/x.dae" concatenates to the correct "/meshes/x.dae".
        return re.sub(r'filename="[^"]*/([^/"]+\.dae)"',
                      r'filename="meshes/\1"', text)

    @app.get("/meshes/{name}")
    def mesh(name: str):
        f = ROOT / "data" / "viz_meshes" / Path(name).name   # no traversal
        if not f.exists():
            raise HTTPException(404)
        return FileResponse(f)

    return app


def main():
    import uvicorn

    app = create_app(WebBridge())
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")


if __name__ == "__main__":
    main()
