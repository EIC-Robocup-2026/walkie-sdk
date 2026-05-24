"use strict";

// ── Tiny helpers ───────────────────────────────────────────────────────
const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => (v === null || v === undefined ? "—" : Number(v).toFixed(d));

let toastTimer = null;
function toast(msg, kind = "") {
  const el = $("toast");
  el.textContent = msg;
  el.className = "toast show " + kind;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => (el.className = "toast"), 2600);
}

async function api(path, { method = "GET", body } = {}) {
  const opts = { method, headers: {} };
  if (body !== undefined) {
    opts.headers["Content-Type"] = "application/json";
    opts.body = JSON.stringify(body);
  }
  const res = await fetch(path, opts);
  let data = null;
  try { data = await res.json(); } catch (_) {}
  if (res.status === 409) throw new Error("Robot not connected");
  if (!res.ok || (data && data.ok === false)) {
    throw new Error((data && data.error) || `HTTP ${res.status}`);
  }
  return data;
}

// Wrap an action button so errors surface as toasts.
function action(fn, okMsg) {
  return async () => {
    try {
      const r = await fn();
      toast(okMsg || "OK", "ok");
      return r;
    } catch (e) {
      toast(e.message, "err");
    }
  };
}

// ── Connection ─────────────────────────────────────────────────────────
$("btn-connect").onclick = async () => {
  $("conn-error").textContent = "";
  const body = {
    ip: $("ip").value.trim(),
    ros_protocol: $("ros_protocol").value,
    ros_port: Number($("ros_port").value),
    camera_protocol: $("camera_protocol").value,
    namespace: $("namespace").value.trim(),
  };
  $("btn-connect").disabled = true;
  $("btn-connect").textContent = "Connecting…";
  try {
    await api("/api/connect", { method: "POST", body });
    toast("Connected", "ok");
  } catch (e) {
    $("conn-error").textContent = e.message;
    toast("Connect failed", "err");
  } finally {
    $("btn-connect").disabled = false;
    $("btn-connect").textContent = "Connect";
    refreshStatus();
  }
};

$("btn-disconnect").onclick = action(async () => {
  stopStream();
  await api("/api/disconnect", { method: "POST" });
  refreshStatus();
}, "Disconnected");

// ── Navigation ─────────────────────────────────────────────────────────
$("btn-goto").onclick = action(
  () => api("/api/nav/goto", {
    method: "POST",
    body: { x: +$("nav_x").value, y: +$("nav_y").value, heading: +$("nav_h").value, blocking: false },
  }),
  "Navigation goal sent"
);
$("btn-cancel").onclick = action(() => api("/api/nav/cancel", { method: "POST" }), "Cancelled");
$("btn-stop").onclick = action(() => api("/api/nav/stop", { method: "POST" }), "STOP sent");

// ── Lift ───────────────────────────────────────────────────────────────
$("lift_slider").oninput = (e) => ($("lift_pos").value = e.target.value);
$("lift_pos").oninput = (e) => ($("lift_slider").value = e.target.value);
$("btn-lift").onclick = action(
  () => api("/api/lift/set", {
    method: "POST",
    body: {
      pos: +$("lift_pos").value,
      speed: +$("lift_speed").value,
      accel: +$("lift_accel").value,
      norm_pos: true,
      blocking: false,
    },
  }),
  "Lift command sent"
);

// ── Arm + Gripper ──────────────────────────────────────────────────────
$("btn-arm-pose").onclick = action(
  () => api("/api/arm/pose", {
    method: "POST",
    body: {
      x: +$("arm_x").value, y: +$("arm_y").value, z: +$("arm_z").value,
      roll: +$("arm_roll").value, pitch: +$("arm_pitch").value, yaw: +$("arm_yaw").value,
      group_name: $("arm_group").value,
      mode: $("arm_mode").value || null,
      blocking: false,
    },
  }),
  "Arm pose sent"
);
$("btn-arm-home").onclick = action(
  () => api("/api/arm/home", { method: "POST", body: { group_name: $("arm_group").value } }),
  "Homing arm"
);

function sendGripper(position) {
  return api("/api/arm/gripper", {
    method: "POST",
    body: { group_name: $("grip_group").value, position, blocking: false },
  });
}
$("btn-grip-open").onclick = action(() => sendGripper(-15.71), "Gripper opening");
$("btn-grip-close").onclick = action(() => sendGripper(0.7), "Gripper closing");
$("btn-grip-set").onclick = action(() => sendGripper(+$("grip_pos").value), "Gripper set");

// ── Camera ─────────────────────────────────────────────────────────────
function startStream() {
  const cam = $("cam_name").value || "head";
  const img = $("cam-img");
  img.src = `/api/camera/stream?camera=${encodeURIComponent(cam)}&t=${Date.now()}`;
  img.style.display = "block";
  $("cam-placeholder").style.display = "none";
}
function stopStream() {
  const img = $("cam-img");
  img.src = "";
  img.style.display = "none";
  $("cam-placeholder").style.display = "block";
}
$("btn-cam-start").onclick = startStream;
$("btn-cam-stop").onclick = stopStream;
$("btn-cam-snap").onclick = () => {
  const cam = $("cam_name").value || "head";
  const img = $("cam-img");
  img.src = `/api/camera/snapshot?camera=${encodeURIComponent(cam)}&t=${Date.now()}`;
  img.style.display = "block";
  $("cam-placeholder").style.display = "none";
};

// ── Status polling ─────────────────────────────────────────────────────
let lastCameras = "";
function applyStatus(s) {
  const connected = !!s.connected;
  $("conn-dot").className = "dot " + (connected ? "on" : "off");
  $("conn-label").textContent = connected
    ? `Connected — ${s.ip || ""} (${s.ros_protocol || ""})`
    : "Disconnected";

  // Enable/disable control cards based on connection.
  ["card-nav", "card-lift", "card-arm", "card-camera", "card-telemetry"].forEach((id) => {
    $(id).classList.toggle("disabled-area", !connected);
  });

  const p = s.position || {};
  $("t-x").textContent = fmt(p.x);
  $("t-y").textContent = fmt(p.y);
  $("t-heading").textContent = fmt(p.heading);
  const v = s.velocity || {};
  $("t-lin").textContent = fmt(v.linear);
  $("t-ang").textContent = fmt(v.angular);
  $("t-nav").textContent = s.nav_status || "—";

  $("lift-norm").textContent = fmt(s.lift, 2);
  $("lift-cm").textContent = fmt(s.lift_cm, 1);
  $("lift-status").textContent = s.lift_status || "";
  if (s.lift !== null && s.lift !== undefined && document.activeElement !== $("lift_slider")) {
    $("lift_slider").value = s.lift;
  }

  // Refresh camera dropdown only when the set changes.
  const cams = (s.cameras && s.cameras.length ? s.cameras : ["head"]).join(",");
  if (cams !== lastCameras) {
    lastCameras = cams;
    const sel = $("cam_name");
    const prev = sel.value;
    sel.innerHTML = "";
    cams.split(",").forEach((name) => {
      const o = document.createElement("option");
      o.value = o.textContent = name;
      sel.appendChild(o);
    });
    if (cams.split(",").includes(prev)) sel.value = prev;
  }
}

async function refreshStatus() {
  try {
    applyStatus(await api("/api/status"));
  } catch (_) {
    applyStatus({ connected: false });
  }
}

refreshStatus();
setInterval(refreshStatus, 1000);
