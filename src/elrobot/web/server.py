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
from contextlib import asynccontextmanager
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
from fastapi.responses import (
    FileResponse,
    HTMLResponse,
    JSONResponse,
    PlainTextResponse,
    StreamingResponse,
)
from fastapi.staticfiles import StaticFiles

from elrobot.calibration import backup
from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_JOINT
from elrobot.web.calib import CalibError, CalibSession
from elrobot.web.collection import CatalogError, CollectionCatalog
from elrobot.web.collection_manager import CollectionError, CollectionManager
from elrobot.web.curation import CuratedReplayLibrary, EpisodeRef
from elrobot.web.export import ExportError, ExportService
from elrobot.web.replay import PhysicalReplay, ReplayError

JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
STALE_S = 1.0
# episode_recorder publishes /record/status at 1 Hz; 3 s means a couple of
# missed ticks before we call it dead rather than flapping on one hiccup.
RECORD_STALE_S = 3.0

# Serial port the CALIBRATION WIZARD opens (nothing else here touches serial).
# Same PORT env knob every other task honours - this arm has already moved
# from ttyACM0 to ttyACM1 once after a replug.
DEFAULT_PORT = os.environ.get("PORT", "/dev/ttyACM0")
DEFAULT_COLLECTION_ROOT = os.environ.get(
    "COLLECTION_ROOT", "data/collections")

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
        self.record_stamp = 0.0
        self.node.create_subscription(
            JointState, "/joint_states", self._on_js, 1)
        for name, topic in (("wrist", "/wrist_cam/image"),
                            ("ext", "/ext_cam/image")):
            self.node.create_subscription(
                Image, topic,
                lambda m, n=name: self._on_img(m, n), 1)
        self.node.create_subscription(String, "/record/status", self._on_record, 1)
        self._pub = self.node.create_publisher(JointState, "/joint_command", 1)
        from rclpy.executors import MultiThreadedExecutor

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._spin = threading.Thread(
            target=self._executor.spin, daemon=True)
        self._spin.start()

    def _on_js(self, msg):
        self.latest_q = dict(zip(msg.name, msg.position))
        self.latest_stamp = time.monotonic()

    def _on_record(self, msg):
        import json
        self.record_status = json.loads(msg.data)
        self.record_stamp = time.monotonic()

    def record_status_fresh(self):
        """None once /record/status goes quiet. The recorder publishes at
        1 Hz; caching the last message forever meant that killing the
        recorder MID-EPISODE left the cockpit showing "recording - N frames"
        indefinitely, with Stop/Discard enabled, while no capture was
        happening. Same staleness discipline as latest_jpeg and latest_q."""
        if self.record_status is None:
            return None
        if time.monotonic() - self.record_stamp > RECORD_STALE_S:
            return None
        return self.record_status

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

    def add_node(self, node):
        self._executor.add_node(node)

    def remove_node(self, node):
        self._executor.remove_node(node)

    def external_recorder_alive(self):
        return any(
            info.node_name == "episode_recorder"
            for info in self.node.get_publishers_info_by_topic(
                "/record/status")
        )


def create_app(bridge, bus_factory=None, port=DEFAULT_PORT,
               backup_root=backup.DEFAULT_ROOT,
               collection_root=DEFAULT_COLLECTION_ROOT,
               recorder_factory=None,
               dataset_validator=None,
               export_service=None) -> FastAPI:
    calib = CalibSession(bus_factory=bus_factory, port=port,
                         backup_root=backup_root)
    catalog = CollectionCatalog(collection_root)
    library = CuratedReplayLibrary(catalog)
    player = PhysicalReplay(
        library,
        publish=bridge.publish_command,
        current_pose=lambda: bridge.latest_q,
        driver_alive=lambda: (bridge.driver_alive()
                              if hasattr(bridge, "driver_alive") else False),
    )
    if recorder_factory is None:
        from elrobot.nodes.episode_recorder import Recorder

        recorder_factory = Recorder
    manager = CollectionManager(
        catalog,
        recorder_factory,
        add_node=getattr(bridge, "add_node", lambda node: None),
        remove_node=getattr(bridge, "remove_node", lambda node: None),
        external_recorder_alive=getattr(
            bridge, "external_recorder_alive", lambda: False),
        dataset_validator=dataset_validator,
    )
    exports = export_service or ExportService(catalog)

    @asynccontextmanager
    async def lifespan(app):
        yield
        manager.shutdown()

    app = FastAPI(title="elrobot cockpit", lifespan=lifespan)
    app.state.bridge = bridge
    app.state.curated_library = library
    app.state.player = player
    app.state.catalog = catalog
    app.state.collection = manager
    app.state.exports = exports

    def coded_error(_, exc):
        return JSONResponse(status_code=exc.code, content={"detail": exc.detail})

    for error in (CalibError, CatalogError, CollectionError, ExportError, ReplayError):
        app.add_exception_handler(error, coded_error)

    def driver_ready() -> bool:
        return bool(
            hasattr(bridge, "driver_alive")
            and bridge.driver_alive()
            and time.monotonic() - bridge.latest_stamp < STALE_S
        )

    @app.post("/api/calib/start")
    def calib_start():
        alive = bridge.driver_alive() if hasattr(bridge, "driver_alive") else False
        return calib.start(alive)

    @app.post("/api/calib/sweep/begin")
    def calib_sweep_begin():
        return calib.sweep_begin()

    @app.post("/api/calib/sweep/end")
    def calib_sweep_end():
        return calib.sweep_end()

    @app.get("/api/calib/state")
    def calib_state():
        return calib.snapshot()

    @app.post("/api/calib/abort")
    def calib_abort():
        # Releases the serial port. Previously the only disconnect was a
        # fully successful finish(), so any mid-session error or change of
        # mind held the bus until the backend was restarted - which blocks
        # the driver (hard rule 3).
        return calib.abort()

    @app.post("/api/calib/eeprom")
    def calib_eeprom(body: dict):
        return calib.eeprom(body.get("confirm", ""))

    @app.post("/api/calib/sign")
    def calib_sign(body: dict):
        return calib.sign(body["joint"], bool(body.get("flip")))

    @app.post("/api/calib/finish")
    def calib_finish(body: dict = None):
        # `out` override exists ONLY so offline tests never touch the real,
        # hand-measured calibration/urdf_ticks.json - the wizard UI never
        # sends it, so real use always writes the real path.
        out = (body or {}).get("out", "calibration/urdf_ticks.json")
        return calib.finish(out=out)

    @app.post("/api/control")
    def control(body: dict):
        want = bool(body.get("on"))
        # Mutual exclusion with physical replay: two publishers on
        # /joint_command with no arbitration is the exact fight the cockpit
        # already warns about, and here both would be automatic.
        if want and player.armed:
            raise HTTPException(
                409, "disarm replay first - it is holding /joint_command")
        if want and not driver_ready():
            raise HTTPException(409, "wait for a fresh driver state first")
        bridge.control_on = want
        return {"control_on": bridge.control_on,
                "seed": dict(bridge.latest_q) if bridge.control_on else {}}

    # ---- managed collection -------------------------------------------

    @app.get("/api/tasks")
    def tasks(include_archived: bool = True):
        return {"tasks": catalog.tasks(include_archived)}

    @app.post("/api/tasks")
    def task_create(body: dict):
        return catalog.create_task(
            body.get("name", ""), body.get("instruction", ""))

    @app.patch("/api/tasks/{task_id}")
    def task_update(task_id: str, body: dict):
        fields = {
            key: body[key]
            for key in ("name", "instruction", "archived")
            if key in body
        }
        return catalog.update_task(task_id, **fields)

    @app.get("/api/collection")
    def collection_state():
        return manager.snapshot()

    @app.post("/api/collection/session/start")
    def collection_session_start(body: dict):
        return manager.start_session(
            body.get("task_id", ""), body.get("name", ""))

    @app.post("/api/collection/episode/start")
    def collection_episode_start(body: dict):
        return manager.start_episode(body.get("task_id", ""))

    @app.post("/api/collection/episode/stop")
    def collection_episode_stop():
        return manager.stop_episode()

    @app.post("/api/collection/episode/discard")
    def collection_episode_discard():
        return manager.discard_episode()

    @app.post("/api/collection/session/finish")
    def collection_session_finish():
        return manager.finish_session()

    @app.get("/api/collection/recovery")
    def collection_recovery():
        return {"sessions": manager.recoveries()}

    @app.post("/api/collection/recovery/{session_id}/finish")
    def collection_recovery_finish(session_id: str):
        return manager.recover_finish(session_id)

    @app.post("/api/collection/recovery/{session_id}/archive")
    def collection_recovery_archive(session_id: str):
        return manager.recover_archive(session_id)

    # ---- immutable curated exports --------------------------------------

    @app.post("/api/exports/preview")
    def export_preview(body: dict):
        return exports.preview(body.get("name", ""), body.get("task_ids", []))

    @app.post("/api/exports")
    def export_start(body: dict):
        return exports.start(body.get("name", ""), body.get("task_ids", []))

    @app.get("/api/exports/{export_id}")
    def export_status(export_id: str):
        return exports.status(export_id)

    # ---- curated collection replay ---------------------------------------

    @app.get("/api/curation/sessions")
    def curation_sessions():
        return {
            "sessions": [
                {
                    "id": session["id"],
                    "name": session["name"],
                    "created_at": session["created_at"],
                    "finalized_at": session["finalized_at"],
                    "episodes": len(session["episodes"]),
                }
                for session in catalog.sessions()
                if session["state"] == "ready"
            ],
        }

    @app.get("/api/curation/sessions/{session_id}/episodes")
    def curation_episodes(session_id: str):
        catalog.session(session_id)
        return {"episodes": library.list_episodes(session_id=session_id)}

    @app.patch("/api/curation/episodes/{session_id}/{source_index}")
    def curation_update(
        session_id: str, source_index: int, body: dict,
    ):
        # Curation changes which frames replay means. Stop and explicitly
        # disarm before changing that meaning so no autonomous publisher can
        # continue against stale selection metadata.
        player.stop()
        player.arm(False, False)
        return catalog.update_episode(session_id, source_index, body)

    @app.get("/api/curation/episodes/{session_id}/{source_index}/states")
    def curation_states(
        session_id: str, source_index: int, raw: bool = False,
    ):
        try:
            return library.states(
                EpisodeRef(session_id, source_index, raw))
        except KeyError as exc:
            raise HTTPException(404, str(exc)) from exc

    @app.get(
        "/api/curation/episodes/{session_id}/{source_index}/frame/{n}",
    )
    def curation_frame(
        session_id: str, source_index: int, n: int,
        cam: str = "wrist", raw: bool = False,
    ):
        if cam not in ("wrist", "ext"):
            raise HTTPException(404, "cam must be wrist or ext")
        try:
            jpeg = library.frame_jpeg(
                EpisodeRef(session_id, source_index, raw), n, cam)
        except KeyError as exc:
            raise HTTPException(404, str(exc)) from exc
        return Response(
            content=jpeg,
            media_type="image/jpeg",
            headers={"Cache-Control": "no-store"},
        )

    # ---- replay ON THE REAL ARM -----------------------------------------
    # These DO move the robot. Everything they publish still goes through
    # elrobot_driver and its gates; the extra locks here are about not
    # starting an autonomous motion by accident.

    @app.get("/api/replay/status")
    def replay_status():
        return player.status()

    @app.post("/api/replay/arm")
    def replay_arm(body: dict):
        return player.arm(bool(body.get("on")), bridge.control_on)

    @app.post("/api/replay/play")
    def replay_play(body: dict):
        try:
            selection = EpisodeRef(
                str(body["session_id"]),
                int(body.get("episode", -1)),
                bool(body.get("raw", False)),
            )
            return player.play(
                selection, float(body.get("speed", 0.6)))
        except (KeyError, ValueError) as e:
            raise HTTPException(404, str(e)) from e

    @app.post("/api/replay/stop")
    def replay_stop():
        return player.stop()

    # Exactly ONE connection may command the arm. control_on lives on the
    # shared bridge, so without this two cockpit tabs would each run their
    # own 25 Hz publisher against the same /joint_command with no arbitration
    # (the two-commander banner cannot see them - commanders() filters by ROS
    # node name and both tabs share this one node). First connection owns
    # control; the rest are monitor-only and told so.
    #
    # The two rules live in named functions on app.state so they can be
    # tested directly: TestClient drives the ASGI app through a single
    # blocking portal and cannot hold two concurrent websocket sessions, so
    # a two-tab scenario is untestable through the transport.
    clients: list[WebSocket] = []

    def client_joined(sock):
        clients.append(sock)

    def may_command(sock) -> bool:
        if bridge.control_on and not driver_ready():
            bridge.control_on = False
        return bool(bridge.control_on and clients and clients[0] is sock)

    def client_gone(sock):
        if sock in clients:
            clients.remove(sock)
        # No cockpit connected -> nobody may hold control. The client's own
        # best-effort POST on unload races page teardown and can lose, which
        # used to leave control_on stuck True with no publisher, so a later
        # tab found the server already "in control".
        if not clients:
            bridge.control_on = False
            player.arm(False, False)

    app.state.clients = clients
    app.state.client_joined = client_joined
    app.state.may_command = may_command
    app.state.client_gone = client_gone

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        client_joined(sock)

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
                    "record": bridge.record_status_fresh(),
                    "collection": manager.snapshot(),
                    "replay": player.status(),
                    # per-connection: is THIS tab the one allowed to command?
                    "is_owner": bool(clients) and clients[0] is sock,
                    "clients": len(clients),
                })
                await asyncio.sleep(1 / 30)

        send_task = asyncio.create_task(sender())
        try:
            while True:
                msg = await sock.receive_json()
                if msg.get("type") == "cmd" and may_command(sock):
                    bridge.publish_command(msg["positions"])
        except WebSocketDisconnect:
            pass
        finally:
            send_task.cancel()
            client_gone(sock)
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

    # no-store on the static assets too, not just / and /urdf. A browser
    # holding a cached app.js against freshly-served HTML silently gives you
    # a page whose new controls are inert - which has now cost real debugging
    # time twice. These are a few KB served over localhost; caching them buys
    # nothing and hides edits.
    @app.middleware("http")
    async def _no_store_static(request, call_next):
        response = await call_next(request)
        if request.url.path.startswith("/static/"):
            response.headers["Cache-Control"] = "no-store"
        return response

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
    print("Cockpit: http://localhost:8080", flush=True)
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")


if __name__ == "__main__":
    main()
