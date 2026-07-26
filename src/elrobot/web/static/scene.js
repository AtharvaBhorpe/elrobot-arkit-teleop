import * as THREE from "three";
import URDFLoader from "/static/vendor/URDFLoader.js";
import { ColladaLoader } from "/static/vendor/three/examples/jsm/loaders/ColladaLoader.js";

export function makeScene(el) {
  const scene = new THREE.Scene();
  const cam = new THREE.PerspectiveCamera(50, 1, 0.01, 10);
  cam.position.set(0.5, 0.35, 0.5);
  cam.up.set(0, 0, 1);                       // ROS Z-up
  const r = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  el.appendChild(r.domElement);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x334, 1.1));
  const key = new THREE.DirectionalLight(0xffffff, 1.4);
  key.position.set(1, 1, 2);
  scene.add(key);
  const grid = new THREE.GridHelper(1.2, 24, 0x262d38, 0x1b2029);
  grid.rotation.x = Math.PI / 2;             // Z-up floor
  scene.add(grid);

  let robot = null;
  const loader = new URDFLoader();
  const dae = new ColladaLoader();
  loader.loadMeshCb = (path, mgr, done) =>
    dae.load(path, (res) => done(res.scene),
      undefined, (err) => console.error("mesh load failed:", path, err));
  loader.load("/urdf", (r2) => { robot = r2; scene.add(robot); },
    undefined, (err) => console.error("URDF load failed:", err));

  // minimal orbit (drag to rotate, wheel to zoom) - no OrbitControls dep
  let drag = null, theta = 0.8, phi = 1.1, dist = 0.75;
  const applyCam = () => {
    cam.position.set(dist * Math.sin(phi) * Math.cos(theta),
                     dist * Math.sin(phi) * Math.sin(theta),
                     dist * Math.cos(phi));
    cam.lookAt(0, 0, 0.15);
  };
  el.addEventListener("pointerdown", (e) => (drag = [e.clientX, e.clientY]));
  addEventListener("pointerup", () => (drag = null));
  addEventListener("pointermove", (e) => {
    if (!drag) return;
    theta -= (e.clientX - drag[0]) * 0.008;
    phi = Math.min(2.6, Math.max(0.3, phi - (e.clientY - drag[1]) * 0.008));
    drag = [e.clientX, e.clientY];
  });
  el.addEventListener("wheel", (e) => {
    dist = Math.min(2, Math.max(0.25, dist + e.deltaY * 0.001));
    e.preventDefault();
  }, { passive: false });

  const resize = () => {
    const w = el.clientWidth, h = el.clientHeight;
    r.setSize(w, h); cam.aspect = w / h; cam.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(el); resize();
  (function loop() {
    applyCam(); r.render(scene, cam); requestAnimationFrame(loop);
  })();

  return { setJoints(q) { if (robot) for (const [n, v] of Object.entries(q))
             robot.setJointValue(n, v); } };
}
