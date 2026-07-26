"""Web cockpit backend: FastAPI + an embedded rclpy node.

An ordinary DDS citizen — subscribes /joint_states and camera images,
publishes /joint_command only while WEB CONTROL is on. The driver keeps
sole serial ownership and ALL safety gates. LAN only, no auth: this page
commands a robot — do not port-forward it.

    pixi run web        # http://<host>:8080
"""

import threading
import time

from fastapi import FastAPI

from elrobot.control.cartesian_ik import ARM_JOINTS, GRIPPER_JOINT

JOINTS = ARM_JOINTS + [GRIPPER_JOINT]
STALE_S = 1.0


class WebBridge:
    """rclpy node wrapper; constructed lazily so tests can use a fake."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        rclpy.init()
        self._JointState = JointState

        class _N(Node):
            pass

        self.node = _N("web_cockpit")
        self.latest_q: dict[str, float] = {}
        self.latest_stamp = 0.0
        self.control_on = False
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self._pub = self.node.create_publisher(JointState, "/joint_command", 1)
        self._spin = threading.Thread(
            target=rclpy.spin, args=(self.node,), daemon=True)
        self._spin.start()

    def _on_js(self, msg):
        self.latest_q = dict(zip(msg.name, msg.position))
        self.latest_stamp = time.monotonic()

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
        alive = bridge.driver_alive() if hasattr(bridge, "driver_alive") else False
        return {
            "driver_alive": alive,
            "control_on": bridge.control_on,
            "commanders": bridge.commanders(),
            "joints": bridge.latest_q,
            "age_s": time.monotonic() - bridge.latest_stamp,
        }

    return app


def main():
    import uvicorn

    app = create_app(WebBridge())
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="warning")


if __name__ == "__main__":
    main()
