"use strict";

// ── Tiny helpers ───────────────────────────────────────────────────────
const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => (v === null || v === undefined ? "—" : Number(v).toFixed(d));
const RAD2DEG = 180 / Math.PI;

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

// ── Head tilt ──────────────────────────────────────────────────────────
$("head_slider").oninput = (e) => ($("head_rad").value = e.target.value);
$("head_rad").oninput = (e) => ($("head_slider").value = e.target.value);
$("btn-head-set").onclick = action(
  () => api("/api/head/tilt", {
    method: "POST",
    body: { angle_rad: +$("head_rad").value },
  }),
  "Head tilt sent"
);
document.querySelectorAll("[data-head]").forEach((btn) => {
  btn.onclick = action(async () => {
    const v = +btn.dataset.head;
    $("head_rad").value = v;
    $("head_slider").value = v;
    return api("/api/head/tilt", { method: "POST", body: { angle_rad: v } });
  }, "Head tilt sent");
});

// ── Arm + Gripper ──────────────────────────────────────────────────────
function armPoseBody() {
  return {
    x: +$("arm_x").value, y: +$("arm_y").value, z: +$("arm_z").value,
    roll: +$("arm_roll").value, pitch: +$("arm_pitch").value, yaw: +$("arm_yaw").value,
    group_name: $("arm_group").value,
    frame_id: $("arm_frame").value.trim() || "base_footprint",
    cartesian_path: $("arm_cartesian").checked,
    blocking: false,
  };
}
$("btn-arm-pose").onclick = action(
  () => api("/api/arm/pose", { method: "POST", body: armPoseBody() }),
  "Arm pose sent"
);
$("btn-arm-relative").onclick = action(
  () => api("/api/arm/pose_relative", {
    method: "POST",
    body: { ...armPoseBody(), ee_frame: $("arm_ee_frame").checked },
  }),
  "Relative move sent"
);
$("btn-arm-home").onclick = action(
  () => api("/api/arm/home", {
    method: "POST",
    body: { group_name: $("arm_group").value, blocking: false },
  }),
  "Homing arm"
);
$("btn-arm-clear").onclick = action(
  () => api("/api/arm/clear_objects", { method: "POST", body: {} }),
  "Cleared collision objects"
);
$("btn-arm-clear-octomap").onclick = action(
  () => api("/api/arm/clear_octomap", { method: "POST", body: {} }),
  "Cleared octomap"
);
let lastEePose = null;
$("btn-arm-ee").onclick = action(async () => {
  const r = await api("/api/arm/ee_pose", {
    method: "POST",
    body: {
      group_name: $("arm_group").value,
      frame_id: $("arm_frame").value.trim() || "base_footprint",
    },
  });
  lastEePose = r.pose || null;
  $("arm-ee-readout").textContent = r.pose
    ? JSON.stringify(r.pose, null, 2)
    : "(no pose)";
  return r;
}, "EE pose read");

// Quaternion -> roll/pitch/yaw (ZYX), matching the commander's tf2 q.setRPY().
function quatToRpy(qx, qy, qz, qw) {
  const sinr = 2 * (qw * qx + qy * qz);
  const cosr = 1 - 2 * (qx * qx + qy * qy);
  const roll = Math.atan2(sinr, cosr);
  let sinp = 2 * (qw * qy - qz * qx);
  sinp = Math.max(-1, Math.min(1, sinp));
  const pitch = Math.asin(sinp);
  const siny = 2 * (qw * qz + qx * qy);
  const cosy = 1 - 2 * (qy * qy + qz * qz);
  const yaw = Math.atan2(siny, cosy);
  return { roll, pitch, yaw };
}

// Fill the X/Y/Z/R/P/Y pose inputs from the last read EE pose.
$("btn-arm-fill-pose").onclick = action(() => {
  if (!lastEePose) throw new Error("Read EE pose first");
  const p = lastEePose;
  $("arm_x").value = (+p.x).toFixed(4);
  $("arm_y").value = (+p.y).toFixed(4);
  $("arm_z").value = (+p.z).toFixed(4);
  const { roll, pitch, yaw } = quatToRpy(p.qx, p.qy, p.qz, p.qw);
  $("arm_roll").value = roll.toFixed(4);
  $("arm_pitch").value = pitch.toFixed(4);
  $("arm_yaw").value = yaw.toFixed(4);
  if ($("arm_frame") && p.frame_id) $("arm_frame").value = p.frame_id;
}, "Pose fields filled");

function sendGripper(positionNorm) {
  return api("/api/arm/gripper", {
    method: "POST",
    body: {
      group_name: $("grip_group").value,
      position: positionNorm,
      norm: true,
      blocking: false,
    },
  });
}
$("btn-grip-open").onclick = action(() => {
  $("grip_pos").value = 1.0;
  return sendGripper(1.0);
}, "Gripper opening");
$("btn-grip-close").onclick = action(() => {
  $("grip_pos").value = 0.0;
  return sendGripper(0.0);
}, "Gripper closing");
$("btn-grip-set").onclick = action(() => sendGripper(+$("grip_pos").value), "Gripper set");
$("btn-grasp-scene-set").onclick = action(() => api("/api/arm/param/set", {
  method: "POST",
  body: { name: "grasp_scene_action", value: $("grasp_scene_action").value },
}), "grasp_scene_action set");
function toggleCollision(enable) {
  return api("/api/arm/toggle_collision", {
    method: "POST",
    body: { group_name: $("collision_group").value, enable },
  });
}
$("btn-collision-enable").onclick = action(() => toggleCollision(true), "Collision enabled");
$("btn-collision-disable").onclick = action(() => toggleCollision(false), "Collision disabled");

// ── Commander params ───────────────────────────────────────────────────
// Coerce the free-text input into a JSON-typed value the way set_param
// expects: true/false → bool, a bare number → number, a comma list → numbers
// (or strings if any element isn't numeric), otherwise a plain string.
function parseParamValue(raw) {
  const s = raw.trim();
  if (s === "true") return true;
  if (s === "false") return false;
  if (s.includes(",")) {
    const parts = s.split(",").map((p) => p.trim());
    const nums = parts.map(Number);
    return nums.every((n) => !Number.isNaN(n)) ? nums : parts;
  }
  const n = Number(s);
  return s !== "" && !Number.isNaN(n) ? n : s;
}
$("btn-planner-set").onclick = action(() => api("/api/arm/param/set", {
  method: "POST",
  body: { name: "planner_id", value: $("planner_id").value },
}), "planner set");
$("btn-param-set").onclick = action(async () => {
  const name = $("param_name").value;
  const value = parseParamValue($("param_value").value);
  const r = await api("/api/arm/param/set", {
    method: "POST",
    body: { name, value },
  });
  $("param-readout").textContent =
    `${name} = ${JSON.stringify(value)}  →  ${r.ok ? "OK" : "REJECTED"}`;
  return r;
}, "Param set");
$("btn-param-get").onclick = action(async () => {
  const name = $("param_name").value;
  const r = await api("/api/arm/param/get", {
    method: "POST",
    body: { name },
  });
  $("param-readout").textContent = `${name} = ${JSON.stringify(r.value)}`;
  if (r.value !== null && r.value !== undefined) {
    $("param_value").value = Array.isArray(r.value)
      ? r.value.join(", ")
      : String(r.value);
  }
  return r;
}, "Param get");

// ── Joint states ───────────────────────────────────────────────────────
$("btn-joints-refresh").onclick = action(async () => {
  const r = await api("/api/joints");
  const j = r.joints || {};
  const names = Object.keys(j).sort();
  $("joints-tag").textContent = `${names.length} joints`;
  $("joints-readout").textContent = names
    .map((n) => {
      const d = j[n];
      return `${n.padEnd(28)} pos=${fmt(d.position, 3)}  vel=${fmt(d.velocity, 3)}  eff=${fmt(d.effort, 3)}`;
    })
    .join("\n");
  return r;
}, "Joints refreshed");

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
  ["card-nav", "card-lift", "card-arm", "card-camera", "card-telemetry", "card-head", "card-joints"].forEach((id) => {
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
  $("t-head").textContent = fmt(s.head_angle);
  $("t-joints").textContent = s.joints_count === null || s.joints_count === undefined ? "—" : s.joints_count;

  $("lift-norm").textContent = fmt(s.lift, 2);
  $("lift-cm").textContent = fmt(s.lift_cm, 1);
  $("lift-status").textContent = s.lift_status || "";
  if (s.lift !== null && s.lift !== undefined && document.activeElement !== $("lift_slider")) {
    $("lift_slider").value = s.lift;
  }

  // Head readout (sync slider only when user isn't dragging it).
  $("head-angle").textContent = fmt(s.head_angle, 3);
  $("head-deg").textContent = s.head_angle === null || s.head_angle === undefined
    ? "—"
    : (s.head_angle * RAD2DEG).toFixed(1);
  if (
    s.head_angle !== null && s.head_angle !== undefined &&
    document.activeElement !== $("head_slider") &&
    document.activeElement !== $("head_rad")
  ) {
    $("head_slider").value = s.head_angle;
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
