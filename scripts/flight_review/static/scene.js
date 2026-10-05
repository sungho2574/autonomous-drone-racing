import * as THREE from "./vendor/three/three.module.min.js";
import { OrbitControls } from "./vendor/three/OrbitControls.js";
import { samplePose } from "./pose.mjs";
export const paths = {
  plan: { name: "Plan", color: "#81c784" },
  actual: { name: "Actual", color: "#ff6b6b" },
  corrected: { name: "Corrected VIO", color: "#4ba3ff" },
  raw: { name: "Raw VIO", color: "#bd94ff" },
};

export class FlightScene {
  constructor(host, controls, status, data) {
    this.host = host;
    this.data = data;
    this.items = {};
    this.disposed = false;
    this.frame = null;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color("#101722");
    this.camera = new THREE.PerspectiveCamera(48, 1, 0.02, 10000);
    this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.domElement.setAttribute("aria-label", "비행 궤적 3D 뷰어");
    host.replaceChildren(this.renderer.domElement);
    controls.replaceChildren();
    status.replaceChildren();
    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.enableRotate = true;
    this.orbit.enablePan = true;
    this.orbit.enableZoom = true;
    this.orbit.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT: THREE.MOUSE.PAN,
    };
    this.orbit.addEventListener("change", () => this.render());
    this.scene.add(new THREE.AmbientLight(0xffffff, 2));
    const light = new THREE.DirectionalLight(0xffffff, 2);
    light.position.set(5, -8, 15);
    this.scene.add(light);
    this.worldAxes = new THREE.AxesHelper(2);
    this.scene.add(this.worldAxes);
    for (const [key, info] of Object.entries(paths)) {
      const rows = data.trajectories[key] || [],
        group = new THREE.Group(),
        vertices = [];
      const maxGap = key === "plan" ? data.plan_max_gap_s : 0.3;
      for (let i = 1; i < rows.length; i++) {
        if (rows[i][0] - rows[i - 1][0] > maxGap) continue;
        vertices.push(...rows[i - 1].slice(1, 4), ...rows[i].slice(1, 4));
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute(
        "position",
        new THREE.Float32BufferAttribute(vertices, 3),
      );
      group.add(
        new THREE.LineSegments(
          geometry,
          new THREE.LineBasicMaterial({
            color: info.color,
            transparent: true,
            opacity: 0.8,
          }),
        ),
      );
      const marker = new THREE.Group();
      marker.add(
        new THREE.Mesh(
          new THREE.SphereGeometry(0.14, 20, 12),
          new THREE.MeshStandardMaterial({ color: info.color, roughness: 0.4 }),
        ),
      );
      const axes = new THREE.AxesHelper(0.7);
      marker.add(axes);
      group.add(marker);
      const lineGeometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(),
        new THREE.Vector3(),
      ]);
      const groundLine = new THREE.Line(
        lineGeometry,
        new THREE.LineDashedMaterial({
          color: info.color,
          dashSize: 0.18,
          gapSize: 0.12,
          transparent: true,
          opacity: 0.85,
        }),
      );
      group.add(groundLine);
      this.scene.add(group);
      const label = document.createElement("label");
      label.className = "series-toggle";
      label.style.setProperty("--series-color", info.color);
      const input = document.createElement("input");
      input.type = "checkbox";
      input.checked = rows.length > 0;
      input.disabled = !rows.length;
      input.setAttribute("aria-label", "3D " + info.name);
      label.append(input, document.createTextNode(info.name));
      controls.append(label);
      const readout = document.createElement("div");
      readout.className = "pose-readout";
      readout.style.color = info.color;
      status.append(readout);
      input.onchange = () => {
        group.visible = input.checked;
        readout.hidden = !input.checked;
        this.render();
      };
      this.items[key] = {
        rows,
        group,
        marker,
        axes,
        groundLine,
        readout,
        maxGap,
      };
      group.visible = input.checked;
    }
    this.addGates(data);
    this.bounds = new THREE.Box3();
    for (const g of data.gates)
      this.bounds.expandByPoint(new THREE.Vector3(g.x, g.y, g.z));
    for (const item of Object.values(this.items))
      for (const r of item.rows)
        this.bounds.expandByPoint(new THREE.Vector3(r[1], r[2], r[3]));
    if (this.bounds.isEmpty())
      this.bounds.set(new THREE.Vector3(-5, -5, 0), new THREE.Vector3(5, 5, 3));
    const size = this.bounds.getSize(new THREE.Vector3());
    const gridSize = Math.max(10, Math.ceil(Math.max(size.x, size.y) * 1.3));
    this.grid = new THREE.GridHelper(
      gridSize,
      Math.min(gridSize, 100),
      0x57657a,
      0x293647,
    );
    this.grid.rotation.x = Math.PI / 2;
    const center = this.bounds.getCenter(new THREE.Vector3());
    this.grid.position.set(center.x, center.y, 0);
    this.scene.add(this.grid);
    this.resize = new ResizeObserver(() => this.resizeView());
    this.resize.observe(host);
    this.resizeView();
    this.fit();
    this.setTime(data.initial_time);
  }
  addGates(data) {
    const outer = data.gate.outer_size,
      inner = data.gate.inner_size,
      bar = (outer - inner) / 2,
      thickness = data.gate.thickness || 0.08;
    for (const g of data.gates) {
      const frame = new THREE.Group();
      frame.position.set(g.x, g.y, g.z);
      frame.rotation.z = (g.yaw_deg * Math.PI) / 180;
      const material = new THREE.MeshStandardMaterial({
        color: 0xf59a40,
        roughness: 0.8,
      });
      for (const side of [-1, 1]) {
        const horizontal = new THREE.Mesh(
          new THREE.BoxGeometry(thickness, outer, bar),
          material,
        );
        horizontal.position.z = (side * (inner + bar)) / 2;
        frame.add(horizontal);
        const vertical = new THREE.Mesh(
          new THREE.BoxGeometry(thickness, bar, inner),
          material,
        );
        vertical.position.y = (side * (inner + bar)) / 2;
        frame.add(vertical);
      }
      this.scene.add(frame);
      const canvas = document.createElement("canvas");
      canvas.width = 128;
      canvas.height = 64;
      const ctx = canvas.getContext("2d");
      ctx.font = "bold 36px sans-serif";
      ctx.fillStyle = "#e9edf5";
      ctx.textAlign = "center";
      ctx.fillText("G" + g.id, 64, 45);
      const sprite = new THREE.Sprite(
        new THREE.SpriteMaterial({
          map: new THREE.CanvasTexture(canvas),
          depthTest: false,
        }),
      );
      sprite.position.set(g.x, g.y, g.z + outer / 2 + 0.4);
      sprite.scale.set(0.9, 0.45, 1);
      this.scene.add(sprite);
    }
  }
  fit(top = false) {
    const box = new THREE.Box3();
    for (const g of this.data.gates) {
      box.expandByPoint(
        new THREE.Vector3(g.x, g.y, g.z - this.data.gate.outer_size / 2),
      );
      box.expandByPoint(
        new THREE.Vector3(g.x, g.y, g.z + this.data.gate.outer_size / 2),
      );
    }
    for (const item of Object.values(this.items))
      if (item.group.visible)
        for (const r of item.rows)
          box.expandByPoint(new THREE.Vector3(r[1], r[2], r[3]));
    if (box.isEmpty()) box.copy(this.bounds);
    const center = box.getCenter(new THREE.Vector3()),
      span = box.getSize(new THREE.Vector3());
    const distance =
      Math.max(5, span.length() / 2) /
      Math.sin((this.camera.fov * Math.PI) / 360) /
      Math.min(1, this.camera.aspect);
    const direction = top
      ? new THREE.Vector3(0, -0.0001, 1)
      : new THREE.Vector3(1, -1, 0.85).normalize();
    this.orbit.target.copy(center);
    this.camera.position
      .copy(center)
      .addScaledVector(direction, distance * 1.15);
    this.camera.lookAt(center);
    this.orbit.update();
    this.render();
  }
  resizeView() {
    const w = this.host.clientWidth,
      h = this.host.clientHeight;
    if (w < 1 || h < 1) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
    this.render();
  }
  setTime(t) {
    this.time = t;
    for (const [key, item] of Object.entries(this.items)) {
      const pose =
        key === "plan" && !this.data.plan_time_aligned
          ? null
          : samplePose(item.rows, t, item.maxGap);
      item.marker.visible = item.groundLine.visible = !!pose;
      if (!pose) {
        item.readout.textContent =
          paths[key].name +
          " · " +
          (key === "plan" && !this.data.plan_time_aligned
            ? "시각 정렬 없음"
            : "해당 시각 데이터 없음");
        continue;
      }
      item.marker.position.fromArray(pose.position);
      item.axes.visible = !!pose.quaternion;
      if (pose.quaternion) item.marker.quaternion.fromArray(pose.quaternion);
      const p = item.groundLine.geometry.attributes.position;
      p.setXYZ(0, pose.position[0], pose.position[1], 0);
      p.setXYZ(1, ...pose.position);
      p.needsUpdate = true;
      item.groundLine.geometry.computeBoundingSphere();
      item.groundLine.computeLineDistances();
      item.readout.textContent =
        paths[key].name +
        " · " +
        pose.position.map((v) => v.toFixed(2)).join(", ") +
        " m";
    }
    this.render();
  }
  render() {
    if (this.disposed || this.frame !== null) return;
    this.frame = requestAnimationFrame(() => {
      this.frame = null;
      if (!this.disposed && this.host.clientWidth && this.host.clientHeight)
        this.renderer.render(this.scene, this.camera);
    });
  }
  dispose() {
    this.disposed = true;
    if (this.frame !== null) cancelAnimationFrame(this.frame);
    this.resize.disconnect();
    this.orbit.dispose();
    const freed = new Set();
    this.scene.traverse((o) => {
      for (const resource of [
        o.geometry,
        ...(Array.isArray(o.material) ? o.material : [o.material]),
      ]) {
        if (resource && !freed.has(resource)) {
          resource.map?.dispose();
          resource.dispose();
          freed.add(resource);
        }
      }
    });
    this.renderer.dispose();
    this.renderer.forceContextLoss();
    this.host.replaceChildren();
  }
}
