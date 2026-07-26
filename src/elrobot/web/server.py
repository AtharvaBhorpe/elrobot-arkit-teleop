"""Web cockpit backend: FastAPI + an embedded rclpy node.

An ordinary DDS citizen — subscribes /joint_states and camera images,
publishes /joint_command only while WEB CONTROL is on. The driver keeps
sole serial ownership and ALL safety gates. LAN only, no auth: this page
commands a robot — do not port-forward it.

    pixi run web        # http://<host>:8080
"""

import asyncio
import threading
import time

import cv2
import numpy as np
from fastapi import (
    FastAPI,
    HTTPException,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import StreamingResponse

from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_JOINT

JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
STALE_S = 1.0


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

        self.node = _N("web_cockpit")
        self.latest_q: dict[str, float] = {}
        self.latest_stamp = 0.0
        self.control_on = False
        self.latest_jpeg: dict[str, tuple[bytes, float]] = {}
        self.node.create_subscription(
            JointState, "/joint_states", self._on_js, 1)
        for name, topic in (("wrist", "/wrist_cam/image"),
                            ("ext", "/ext_cam/image")):
            self.node.create_subscription(
                Image, topic,
                lambda m, n=name: self._on_img(m, n), 1)
        self._pub = self.node.create_publisher(JointState, "/joint_command", 1)
        self._spin = threading.Thread(
            target=rclpy.spin, args=(self.node,), daemon=True)
        self._spin.start()

    def _on_js(self, msg):
        self.latest_q = dict(zip(msg.name, msg.position))
        self.latest_stamp = time.monotonic()

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


def create_app(bridge) -> FastAPI:
    app = FastAPI(title="elrobot cockpit")
    app.state.bridge = bridge

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

    @app.post("/api/control")
    def control(body: dict):
        bridge.control_on = bool(body.get("on"))
        return {"control_on": bridge.control_on,
                "seed": dict(bridge.latest_q) if bridge.control_on else {}}

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

    @app.get("/cam/{name}")
    def cam(name: str):
        if name not in ("wrist", "ext"):
            raise HTTPException(404)

        def gen():
            count = 0
            max_frames = 1000  # ponytail: limit for testing; real stream
            # is infinite but client-driven by browser consumption
            while count < max_frames:
                count += 1
                jpeg, ts = getattr(bridge, "latest_jpeg", {}).get(
                    name, (None, 0.0))
                if jpeg is None or time.monotonic() - ts > 2.0:
                    jpeg = _placeholder(name)
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + jpeg + b"\r\n")

        return StreamingResponse(
            gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    return app


def main():
    import uvicorn

    app = create_app(WebBridge())
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")


if __name__ == "__main__":
    main()
