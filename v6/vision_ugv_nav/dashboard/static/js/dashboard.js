// Vision UGV Mission Control Dashboard JavaScript
// Dynamic Auto-Fit Radar Viewport, Live Waypoint Previews & In-Flight Retargeting

const radarCanvas = document.getElementById('radar-canvas');
const radarCtx = radarCanvas ? radarCanvas.getContext('2d') : null;
const liveCamImg = document.getElementById('live-camera-img');
const quadGrid = document.getElementById('quad-camera-grid');
const eventLogContainer = document.getElementById('event-log-container');
const modalLogContainer = document.getElementById('modal-log-container');

let latestTelemetry = {};
let latestSupervisor = {};
let radarSweepAngle = 0.0;
let reticlePulsePhase = 0.0;
let loggedEvents = new Set();
let prevStatus = '';
let prevReplanCount = 0;
let activeCam = 'fwd';
let radarViewMode = 'radar'; // 'radar' or 'hazard'
let viewportMode = 'fit';    // 'fit' or 'chase'

// Live Waypoint States
let activeStart = { x: -10.0, y: -8.0 };
let activeGoal = { x: 10.0, y: 8.0 };
let previewStart = { x: -10.0, y: -8.0 };
let previewGoal = { x: 10.0, y: 8.0 };
let isPreviewDirty = false;
let lastDispatchTs = 0;

function parseCoord(val, fallback) {
  const n = parseFloat(val);
  return isNaN(n) ? fallback : n;
}

// 1. Dynamic Canvas Sizing & DPR Sync
function resizeRadarCanvas() {
  const container = document.getElementById('radar-canvas-container');
  if (container && radarCanvas) {
    const rect = container.getBoundingClientRect();
    const displayW = Math.floor(rect.width);
    const displayH = Math.floor(rect.height);
    if (displayW > 50 && displayH > 50) {
      if (radarCanvas.width !== displayW || radarCanvas.height !== displayH) {
        radarCanvas.width = displayW;
        radarCanvas.height = displayH;
      }
    }
  }
}
window.addEventListener('resize', resizeRadarCanvas);
setTimeout(resizeRadarCanvas, 100);

// 2. Viewport Coordinate Transformation (Never Goes Out Of Frame!)
function toCanvas(wx, wy) {
  if (!radarCanvas) return [0, 0];
  const w = radarCanvas.width;
  const h = radarCanvas.height;

  const pose = latestTelemetry.pose || { x: -10.0, y: -8.0 };
  const rx = pose.x || -10.0;
  const ry = pose.y || -8.0;

  let cenX, cenY, spanX, spanY;

  if (viewportMode === 'chase') {
    // Center tightly on moving UGV with 14m local surround
    cenX = rx;
    cenY = ry;
    spanX = 24.0;
    spanY = 18.0;
  } else {
    // 'fit' mode: Encompasses Start, Goal, UGV, Previews & Bounds with generous padding
    const xs = [activeStart.x, activeGoal.x, previewStart.x, previewGoal.x, rx, -11.0, 11.0];
    const ys = [activeStart.y, activeGoal.y, previewStart.y, previewGoal.y, ry, -9.5, 9.5];

    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);

    cenX = (minX + maxX) / 2.0;
    cenY = (minY + maxY) / 2.0;

    // Add at least 3.5m border margin so circles & badges never clip
    spanX = Math.max(26.0, (maxX - minX) + 6.0);
    spanY = Math.max(20.0, (maxY - minY) + 5.0);
  }

  // Uniform scaling guarantees perfect circles without stretching into ovals
  const scale = Math.min((w - 24) / spanX, (h - 24) / spanY);

  const cx = w / 2.0 + (wx - cenX) * scale;
  const cy = h / 2.0 - (wy - cenY) * scale;

  return [cx, cy, scale];
}

// 3. Multi-Camera Feed Switcher
function switchCamera(cam) {
  activeCam = cam;
  document.querySelectorAll('.cam-tab').forEach(t => {
    t.classList.toggle('active', t.dataset.cam === cam);
  });

  const hudCamInfo = document.getElementById('hud-cam-info');
  if (cam === 'quad') {
    if (liveCamImg) liveCamImg.style.display = 'none';
    if (quadGrid) quadGrid.style.display = 'grid';
    if (hudCamInfo) hudCamInfo.innerText = 'QUAD SURROUND VIEW | 4 CAMERAS SYNCHRONIZED';
  } else {
    if (quadGrid) quadGrid.style.display = 'none';
    if (liveCamImg) liveCamImg.style.display = 'block';
    if (hudCamInfo) {
      if (cam === 'fwd') hudCamInfo.innerText = 'CAM 1: FORWARD RGB-D | YOLOv8 OBJECT CLASSIFIER';
      else if (cam === 'midas') hudCamInfo.innerText = 'CAM 1: MiDaS MONOCULAR RELATIVE DEPTH (INFERNO)';
      else if (cam === 'left') hudCamInfo.innerText = 'CAM 2: LEFT FLANK (+45° FOV)';
      else if (cam === 'right') hudCamInfo.innerText = 'CAM 3: RIGHT FLANK (-45° FOV)';
    }
  }
}

// Live Camera Feed Refresh Loop (8-10Hz)
function refreshLiveCamera() {
  const ts = Date.now();
  if (activeCam === 'quad') {
    const fwdImg = document.getElementById('quad-img-fwd');
    const midasImg = document.getElementById('quad-img-midas');
    const leftImg = document.getElementById('quad-img-left');
    const rightImg = document.getElementById('quad-img-right');
    if (fwdImg) fwdImg.src = '/static/img/live_camera.jpg?t=' + ts;
    if (midasImg) midasImg.src = '/static/img/midas_depth.jpg?t=' + ts;
    if (leftImg) leftImg.src = '/static/img/live_camera_left.jpg?t=' + ts;
    if (rightImg) rightImg.src = '/static/img/live_camera_right.jpg?t=' + ts;
  } else if (liveCamImg) {
    let src = '/static/img/live_camera.jpg';
    if (activeCam === 'midas') src = '/static/img/midas_depth.jpg';
    else if (activeCam === 'left') src = '/static/img/live_camera_left.jpg';
    else if (activeCam === 'right') src = '/static/img/live_camera_right.jpg';

    const nextSrc = src + '?t=' + ts;
    const preload = new Image();
    preload.onload = function() {
      liveCamImg.src = nextSrc;
    };
    preload.src = nextSrc;
  }
}
setInterval(refreshLiveCamera, 125);

function handleCamImgError(img) {
  // Wait until controller writes first frame
}

// 4. Telemetry Polling Loop (10Hz)
async function fetchTelemetry() {
  try {
    const res = await fetch('/api/telemetry');
    const data = await res.json();
    latestTelemetry = data.telemetry || {};
    latestSupervisor = data.supervisor || {};

    if (latestTelemetry.goal && (Date.now() - lastDispatchTs > 1500)) {
      activeGoal.x = latestTelemetry.goal.x;
      activeGoal.y = latestTelemetry.goal.y;
    }

    updateDashboardUI(latestTelemetry, latestSupervisor);
    processEventLogs(latestTelemetry, latestSupervisor);
  } catch (err) {
    // Retry silently
  }
}
setInterval(fetchTelemetry, 100);

// 5. Dashboard UI Updates
function updateDashboardUI(tel, sup) {
  const time = tel.time || sup.sim_time || 0.0;
  document.getElementById('val-time').innerText = time.toFixed(1) + 's';

  const status = tel.status || 'STANDBY';
  const badge = document.getElementById('badge-status');
  badge.innerText = status;
  if (status.includes('EMERGENCY') || status.includes('OBSTACLE') || status.includes('REPLAN')) {
    badge.className = 'badge status-badge alert';
  } else if (status.includes('GOAL')) {
    badge.className = 'badge status-badge success';
  } else {
    badge.className = 'badge status-badge';
  }

  // Kinematics Bar
  const pose = tel.pose || { v: 0, w: 0, theta_deg: 0 };
  document.getElementById('val-speed').innerText = Math.abs(pose.v || 0).toFixed(2);
  document.getElementById('bar-speed').style.width = Math.min(100, (Math.abs(pose.v || 0) / 0.85) * 100) + '%';

  const heading = pose.theta_deg || 0;
  document.getElementById('val-heading').innerText = heading.toFixed(1);
  document.getElementById('bar-heading').style.width = (((heading + 180) % 360) / 360) * 100 + '%';

  const distGoal = tel.dist_to_goal !== undefined ? tel.dist_to_goal : 26.0;
  document.getElementById('val-dist-goal').innerText = distGoal.toFixed(2);
  document.getElementById('bar-goal').style.width = Math.max(0, Math.min(100, 100 - (distGoal / 26.0) * 100)) + '%';

  // Localization RMSE & Collisions
  const rmse = sup.loc_rmse || 0;
  document.getElementById('val-rmse').innerText = rmse.toFixed(3);

  const collisions = sup.collisions || 0;
  const colElem = document.getElementById('val-collisions');
  colElem.innerText = collisions;
  colElem.className = collisions > 0 ? 'k-val text-danger' : 'k-val text-success';

  const replans = tel.replan_count || 0;
  document.getElementById('val-replans').innerText = replans;

  // Corridor Alert
  const corridorAlert = document.getElementById('corridor-alert');
  if (tel.corridor_blocked) {
    corridorAlert.innerText = 'HAZARD: ' + (tel.min_corridor_dist ? tel.min_corridor_dist.toFixed(1) + 'm' : 'AHEAD');
    corridorAlert.style.color = '#ef4444';
  } else {
    corridorAlert.innerText = 'CORRIDOR: CLEAR';
    corridorAlert.style.color = '#34d399';
  }

  // YOLO HUD
  const yoloDets = tel.yolo_detections || [];
  const closestObsElem = document.getElementById('hud-closest-obs');
  if (yoloDets.length > 0) {
    const top = yoloDets[0];
    closestObsElem.innerText = `YOLO: ${top.class} (${top.distance.toFixed(1)}m)`;
  } else {
    closestObsElem.innerText = 'YOLO: 0 OBJECTS';
  }

  // Update in-flight retarget readiness
  const isMoving = Math.abs(pose.v || 0) > 0.05 || status === 'NAVIGATING';
  const tagRetarget = document.getElementById('retarget-status-badge');
  if (tagRetarget) {
    if (isMoving) {
      tagRetarget.innerText = '⚡ IN-FLIGHT MOVING';
      tagRetarget.className = 'tag tag-blue';
    } else {
      tagRetarget.innerText = '🟢 MISSION READY';
      tagRetarget.className = 'tag tag-green';
    }
  }
}

// 6. Real-Time Detection & Event Logging
function processEventLogs(tel, sup) {
  const time = tel.time || sup.sim_time || 0.0;
  const timeStr = `[${time.toFixed(1).padStart(4, '0')}s]`;

  const status = tel.status || '';
  if (status && status !== prevStatus && time > 0.5) {
    if (status.includes('REPLAN')) {
      addLogEntry(`${timeStr} ⚡ THETA* REPLAN #${tel.replan_count}: Any-angle evasive spline recomputed around hazard`, 'log-warn');
    } else if (status.includes('OBSTACLE')) {
      addLogEntry(`${timeStr} ⚠️ CORRIDOR BLOCKED at ${tel.min_corridor_dist ? tel.min_corridor_dist.toFixed(1) + 'm' : 'close range'}! Initiating instant evasion`, 'log-hazard');
    } else if (status.includes('EMERGENCY')) {
      addLogEntry(`${timeStr} 🛑 EMERGENCY BRAKE: Safety buffer breach`, 'log-hazard');
    } else if (status.includes('GOAL')) {
      addLogEntry(`${timeStr} 🏆 MISSION COMPLETE! Precision touchdown on Goal Beacon B.`, 'log-goal');
    }
    prevStatus = status;
  }

  if (tel.replan_count > prevReplanCount) {
    prevReplanCount = tel.replan_count;
  }

  const yoloDets = tel.yolo_detections || [];
  for (const det of yoloDets) {
    const distBin = Math.round(det.distance);
    const logKey = `${det.class}_${distBin}m`;
    if (!loggedEvents.has(logKey) && det.distance < 5.5) {
      loggedEvents.add(logKey);
      let clsName = 'log-info';
      if (det.class === 'HAZARD_BARREL') clsName = 'log-hazard';
      else if (det.class === 'ROCK') clsName = 'log-obs';
      else if (det.class === 'GOAL_BEACON') clsName = 'log-goal';

      addLogEntry(`${timeStr} 👁 YOLO DETECT: [${det.class}] at ${det.distance.toFixed(1)}m (Conf: ${Math.round(det.confidence * 100)}%)`, clsName);
    }
  }

  if (sup.collisions > 0) {
    const colKey = `col_${sup.collisions}`;
    if (!loggedEvents.has(colKey)) {
      loggedEvents.add(colKey);
      addLogEntry(`${timeStr} 💥 COLLISION IMPACT #${sup.collisions} DETECTED!`, 'log-hazard');
    }
  }
}

function addLogEntry(text, className = 'log-info') {
  if (eventLogContainer) {
    const row = document.createElement('div');
    row.className = `log-entry ${className}`;
    row.innerText = text;
    eventLogContainer.appendChild(row);

    while (eventLogContainer.children.length > 80) {
      eventLogContainer.removeChild(eventLogContainer.firstChild);
    }
    eventLogContainer.scrollTop = eventLogContainer.scrollHeight;
  }

  if (modalLogContainer) {
    const mRow = document.createElement('div');
    mRow.className = `log-entry ${className}`;
    mRow.innerText = text;
    modalLogContainer.appendChild(mRow);
    modalLogContainer.scrollTop = modalLogContainer.scrollHeight;
  }
}

function openLogModal() {
  const modal = document.getElementById('log-modal');
  if (modal) {
    modal.style.display = 'flex';
    if (modalLogContainer && eventLogContainer) {
      modalLogContainer.innerHTML = eventLogContainer.innerHTML;
      modalLogContainer.scrollTop = modalLogContainer.scrollHeight;
    }
  }
}

function closeLogModal() {
  const modal = document.getElementById('log-modal');
  if (modal) modal.style.display = 'none';
}

// 7. Radar Controls: View Modes & Viewport Modes
function setRadarView(mode) {
  radarViewMode = mode;
  document.getElementById('btn-view-radar').classList.toggle('active', mode === 'radar');
  document.getElementById('btn-view-hazard').classList.toggle('active', mode === 'hazard');
}

function setViewportMode(mode) {
  viewportMode = mode;
  document.getElementById('btn-zoom-fit').classList.toggle('active', mode === 'fit');
  document.getElementById('btn-zoom-chase').classList.toggle('active', mode === 'chase');
}

// 8. Waypoint Input Listeners & Real-Time Previews
function onWaypointInputChange() {
  const sx = parseFloat(document.getElementById('inp-start-x').value);
  const sy = parseFloat(document.getElementById('inp-start-y').value);
  const gx = parseFloat(document.getElementById('inp-goal-x').value);
  const gy = parseFloat(document.getElementById('inp-goal-y').value);

  if (!isNaN(sx) && !isNaN(sy)) {
    previewStart.x = sx;
    previewStart.y = sy;
  }
  if (!isNaN(gx) && !isNaN(gy)) {
    previewGoal.x = gx;
    previewGoal.y = gy;
  }

  // Check if preview differs from active established mission
  const diffGoal = Math.hypot(previewGoal.x - activeGoal.x, previewGoal.y - activeGoal.y) > 0.3;
  const diffStart = Math.hypot(previewStart.x - activeStart.x, previewStart.y - activeStart.y) > 0.3;
  isPreviewDirty = diffGoal || diffStart;

  const startBadge = document.getElementById('badge-start-state');
  const goalBadge = document.getElementById('badge-goal-state');

  if (startBadge) {
    startBadge.innerText = isPreviewDirty ? 'PREVIEW' : 'ACTIVE';
    startBadge.className = isPreviewDirty ? 'coord-badge preview' : 'coord-badge';
  }
  if (goalBadge) {
    goalBadge.innerText = isPreviewDirty ? 'NEW TARGET' : 'TARGET';
    goalBadge.className = isPreviewDirty ? 'coord-badge preview' : 'coord-badge target';
  }
}

function setPreset(sx, sy, gx, gy, btnElem) {
  document.getElementById('inp-start-x').value = sx;
  document.getElementById('inp-start-y').value = sy;
  document.getElementById('inp-goal-x').value = gx;
  document.getElementById('inp-goal-y').value = gy;

  document.querySelectorAll('.btn-preset').forEach(b => b.classList.remove('active'));
  if (btnElem) btnElem.classList.add('active');

  // Immediately dispatch upon preset click for seamless in-flight retargeting
  dispatchMission();
}

async function dispatchMission() {
  const sx = parseCoord(document.getElementById('inp-start-x').value, -10.0);
  const sy = parseCoord(document.getElementById('inp-start-y').value, -8.0);
  const gx = parseCoord(document.getElementById('inp-goal-x').value, 10.0);
  const gy = parseCoord(document.getElementById('inp-goal-y').value, 8.0);

  const isTeleport = document.getElementById('rad-teleport') ? document.getElementById('rad-teleport').checked : false;
  const resetRobot = isTeleport;

  activeStart.x = sx;
  activeStart.y = sy;
  previewStart.x = sx;
  previewStart.y = sy;
  activeGoal.x = gx;
  activeGoal.y = gy;
  previewGoal.x = gx;
  previewGoal.y = gy;
  isPreviewDirty = false;
  lastDispatchTs = Date.now();

  const startBadge = document.getElementById('badge-start-state');
  const goalBadge = document.getElementById('badge-goal-state');
  if (startBadge) { startBadge.innerText = 'ACTIVE'; startBadge.className = 'coord-badge'; }
  if (goalBadge) { goalBadge.innerText = 'TARGET'; goalBadge.className = 'coord-badge target'; }

  try {
    const res = await fetch('/api/set_mission', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        start_x: sx,
        start_y: sy,
        goal_x: gx,
        goal_y: gy,
        reset_robot: resetRobot
      })
    });
    const data = await res.json();
    if (resetRobot) {
      addLogEntry(`[${(latestTelemetry.time || 0).toFixed(1)}s] 🔁 RESTART MISSION: UGV repositioned to Start (${sx}, ${sy}) → Goal (${gx}, ${gy})`, 'log-warn');
    } else {
      addLogEntry(`[${(latestTelemetry.time || 0).toFixed(1)}s] ⚡ IN-FLIGHT RETARGET: UGV dynamically steering to new Goal (${gx}, ${gy}) via Theta* replan`, 'log-warn');
    }
  } catch (err) {
    console.error('Failed to dispatch mission:', err);
  }
}

// Attach Enter key triggers to waypoint inputs for instantaneous dispatch
['inp-start-x', 'inp-start-y', 'inp-goal-x', 'inp-goal-y'].forEach(id => {
  const el = document.getElementById(id);
  if (el) {
    el.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        dispatchMission();
      }
    });
  }
});

// 9. Advanced Tactical Vector Radar with Compass Tape, FOV Cone & Threat HUD
function renderRadarFrame() {
  if (!radarCanvas || !radarCtx) {
    requestAnimationFrame(renderRadarFrame);
    return;
  }

  resizeRadarCanvas();

  const w = radarCanvas.width;
  const h = radarCanvas.height;
  radarCtx.clearRect(0, 0, w, h);

  // Tactical Deep Slate Background
  radarCtx.fillStyle = '#060a12';
  radarCtx.fillRect(0, 0, w, h);

  const pose = latestTelemetry.pose || { x: -10, y: -8, theta_deg: 45 };
  const [rx, ry, scale] = toCanvas(pose.x, pose.y);

  // A. Dynamic Dirt Path Corridor (Curved Track or Custom Direct Path)
  const trackWaypoints = [
    [-10.0, -8.0], [-8.5, -7.0], [-5.5, -4.75], [-2.5, -2.25],
    [0.5, 0.5], [3.5, 3.25], [6.5, 5.5], [9.0, 7.25], [10.0, 8.0]
  ];

  const isDefaultTrack = Math.hypot(activeStart.x - (-10), activeStart.y - (-8)) < 1.5 &&
                         Math.hypot(activeGoal.x - 10, activeGoal.y - 8) < 1.5;

  const roadPath = isDefaultTrack ? trackWaypoints : [[activeStart.x, activeStart.y], [activeGoal.x, activeGoal.y]];

  if (roadPath.length >= 2) {
    // 1. Broad Dirt Road Surface
    radarCtx.strokeStyle = 'rgba(180, 83, 9, 0.22)';
    radarCtx.lineWidth = Math.max(14, scale * 2.4);
    radarCtx.lineCap = 'round';
    radarCtx.lineJoin = 'round';
    radarCtx.beginPath();
    const [rw0x, rw0y] = toCanvas(roadPath[0][0], roadPath[0][1]);
    radarCtx.moveTo(rw0x, rw0y);
    for (let i = 1; i < roadPath.length; i++) {
      const [rwx, rwy] = toCanvas(roadPath[i][0], roadPath[i][1]);
      radarCtx.lineTo(rwx, rwy);
    }
    radarCtx.stroke();

    // 2. Road Edges / Curbs
    radarCtx.strokeStyle = 'rgba(234, 179, 8, 0.35)';
    radarCtx.lineWidth = 1.5;
    radarCtx.setLineDash([6, 5]);
    radarCtx.stroke();
    radarCtx.setLineDash([]);
  }

  // B. Smooth Explored Path Ribbon & Threat Auras (if 'hazard' view enabled)
  if (radarViewMode === 'hazard') {
    const estTraj = latestTelemetry.estimated_trajectory || [];
    if (estTraj.length > 1) {
      radarCtx.strokeStyle = 'rgba(16, 185, 129, 0.18)';
      radarCtx.lineWidth = scale * 1.1;
      radarCtx.lineCap = 'round';
      radarCtx.lineJoin = 'round';
      radarCtx.beginPath();
      const [h0x, h0y] = toCanvas(estTraj[0][0], estTraj[0][1]);
      radarCtx.moveTo(h0x, h0y);
      for (let i = 1; i < estTraj.length; i++) {
        const [hx, hy] = toCanvas(estTraj[i][0], estTraj[i][1]);
        radarCtx.lineTo(hx, hy);
      }
      radarCtx.stroke();
    }

    const allObstacles = [
      [-4.8, -3.8], [0.0, 0.0], [4.5, 4.0],
      [-8.5, -4.5], [-5.5, -7.5], [-5.0, -1.8], [-2.5, -4.8],
      [-2.0, 0.8], [0.8, -2.5], [1.0, 3.8], [3.8, 0.8],
      [3.5, 6.0], [6.5, 3.0], [6.0, 7.8], [8.5, 4.0]
    ];
    allObstacles.forEach(([ox, oy]) => {
      const [ocx, ocy] = toCanvas(ox, oy);
      const radR = scale * 1.6;
      const grad = radarCtx.createRadialGradient(ocx, ocy, 2, ocx, ocy, radR);
      grad.addColorStop(0, 'rgba(239, 68, 68, 0.65)');
      grad.addColorStop(0.5, 'rgba(245, 158, 11, 0.25)');
      grad.addColorStop(1, 'rgba(245, 158, 11, 0.0)');
      radarCtx.fillStyle = grad;
      radarCtx.beginPath();
      radarCtx.arc(ocx, ocy, radR, 0, 2 * Math.PI);
      radarCtx.fill();
    });
  }

  // C. Concentric Range Rings with Tactical Distance Annotations & Cardinal Compass
  const [centerMapX, centerMapY] = toCanvas(0, 0);
  radarCtx.strokeStyle = 'rgba(56, 189, 248, 0.12)';
  radarCtx.lineWidth = 1;
  [5, 10, 15, 20].forEach(r => {
    radarCtx.beginPath();
    radarCtx.arc(centerMapX, centerMapY, r * scale, 0, 2 * Math.PI);
    radarCtx.stroke();

    // Range distance tags along axes
    radarCtx.fillStyle = 'rgba(56, 189, 248, 0.55)';
    radarCtx.font = '8.5px "JetBrains Mono", monospace';
    radarCtx.fillText(`${r}m`, centerMapX + r * scale + 2, centerMapY - 3);
    radarCtx.fillText(`${r}m`, centerMapX + 3, centerMapY - r * scale + 10);
  });

  // Crosshairs & Cardinal Labels
  radarCtx.strokeStyle = 'rgba(56, 189, 248, 0.15)';
  radarCtx.setLineDash([4, 4]);
  radarCtx.beginPath();
  radarCtx.moveTo(centerMapX, 0);
  radarCtx.lineTo(centerMapX, h);
  radarCtx.moveTo(0, centerMapY);
  radarCtx.lineTo(w, centerMapY);
  radarCtx.stroke();
  radarCtx.setLineDash([]);

  radarCtx.fillStyle = 'rgba(56, 189, 248, 0.75)';
  radarCtx.font = 'bold 9.5px "JetBrains Mono", monospace';
  radarCtx.fillText('N', centerMapX - 4, Math.max(14, centerMapY - 20 * scale - 4));
  radarCtx.fillText('S', centerMapX - 4, Math.min(h - 6, centerMapY + 20 * scale + 13));
  radarCtx.fillText('E', Math.min(w - 14, centerMapX + 20 * scale + 5), centerMapY + 3);
  radarCtx.fillText('W', Math.max(4, centerMapX - 20 * scale - 14), centerMapY + 3);

  // D. Static Roadblock Boulders & Proximity Awareness
  const centerRocks = [[-4.8, -3.8], [0.0, 0.0], [4.5, 4.0]];
  centerRocks.forEach(([cx, cy]) => {
    const [rcx, rcy] = toCanvas(cx, cy);
    const distToUgv = Math.hypot(cx - pose.x, cy - pose.y);

    radarCtx.fillStyle = '#ea580c';
    radarCtx.strokeStyle = distToUgv < 4.0 ? '#ef4444' : '#fb923c';
    radarCtx.lineWidth = 2;
    radarCtx.beginPath();
    radarCtx.arc(rcx, rcy, Math.max(5, scale * 0.42), 0, 2 * Math.PI);
    radarCtx.fill();
    radarCtx.stroke();

    // Threat label with distance
    radarCtx.fillStyle = distToUgv < 4.0 ? '#fca5a5' : '#fed7aa';
    radarCtx.font = 'bold 8px "JetBrains Mono", monospace';
    const tag = distToUgv < 5.0 ? `ROCK ${distToUgv.toFixed(1)}m` : 'ROCK';
    radarCtx.fillText(tag, rcx - 14, rcy - 8);
  });

  // Side Natural Boulders
  const sideRocks = [
    [-8.5, -4.5], [-5.5, -7.5], [-5.0, -1.8], [-2.5, -4.8],
    [-2.0, 0.8], [0.8, -2.5], [1.0, 3.8], [3.8, 0.8],
    [3.5, 6.0], [6.5, 3.0], [6.0, 7.8], [8.5, 4.0]
  ];
  radarCtx.fillStyle = '#475569';
  sideRocks.forEach(([ox, oy]) => {
    const [ocx, ocy] = toCanvas(ox, oy);
    radarCtx.beginPath();
    radarCtx.arc(ocx, ocy, Math.max(3, scale * 0.28), 0, 2 * Math.PI);
    radarCtx.fill();
  });

  // Dynamic Injected Hazard Barrels
  const detectedDynObs = latestTelemetry.obstacles || [];
  for (const dobs of detectedDynObs) {
    const [dx, dy] = toCanvas(dobs.x, dobs.y);
    radarCtx.fillStyle = '#ef4444';
    radarCtx.strokeStyle = '#fca5a5';
    radarCtx.lineWidth = 2;
    radarCtx.beginPath();
    radarCtx.arc(dx, dy, Math.max(6, scale * 0.45), 0, 2 * Math.PI);
    radarCtx.fill();
    radarCtx.stroke();

    const dStr = dobs.distance ? `${dobs.distance.toFixed(1)}m` : 'HAZARD';
    radarCtx.fillStyle = '#fee2e2';
    radarCtx.font = 'bold 8.5px "JetBrains Mono", monospace';
    radarCtx.fillText(`⚠️ ${dStr}`, dx - 16, dy - 9);
  }

  // E. Established Start Waypoint (Point A)
  const [pStart_x, pStart_y] = toCanvas(activeStart.x, activeStart.y);
  radarCtx.strokeStyle = '#10b981';
  radarCtx.lineWidth = 2;
  radarCtx.beginPath();
  radarCtx.arc(pStart_x, pStart_y, 9, 0, 2 * Math.PI);
  radarCtx.stroke();
  radarCtx.fillStyle = '#10b981';
  radarCtx.font = 'bold 10px Rajdhani';
  radarCtx.fillText(`START (A) [${activeStart.x.toFixed(1)}, ${activeStart.y.toFixed(1)}]`, pStart_x - 30, pStart_y - 12);

  // F. Established Target Goal (Point B)
  const [pEnd_x, pEnd_y] = toCanvas(activeGoal.x, activeGoal.y);
  reticlePulsePhase += 0.05;
  const pulseR = 12 + Math.sin(reticlePulsePhase) * 2.5;

  radarCtx.strokeStyle = '#06b6d4';
  radarCtx.fillStyle = 'rgba(6, 182, 212, 0.2)';
  radarCtx.lineWidth = 2;
  radarCtx.beginPath();
  radarCtx.arc(pEnd_x, pEnd_y, pulseR, 0, 2 * Math.PI);
  radarCtx.fill();
  radarCtx.stroke();

  // Crosshair ticks
  radarCtx.beginPath();
  radarCtx.moveTo(pEnd_x - pulseR - 3, pEnd_y); radarCtx.lineTo(pEnd_x + pulseR + 3, pEnd_y);
  radarCtx.moveTo(pEnd_x, pEnd_y - pulseR - 3); radarCtx.lineTo(pEnd_x, pEnd_y + pulseR + 3);
  radarCtx.stroke();

  radarCtx.fillStyle = '#38bdf8';
  radarCtx.font = 'bold 10.5px Rajdhani';
  const goalDist = latestTelemetry.dist_to_goal !== undefined ? `${latestTelemetry.dist_to_goal.toFixed(1)}m` : '';
  radarCtx.fillText(`GOAL (B) [${activeGoal.x.toFixed(1)}, ${activeGoal.y.toFixed(1)}] ${goalDist}`, pEnd_x - 32, pEnd_y - 15);

  // G. Live Interactive Waypoint Preview (When numbers changed or preset clicked!)
  if (isPreviewDirty) {
    const [pgx, pgy] = toCanvas(previewGoal.x, previewGoal.y);

    // Pending Goal Holographic Reticle (Amber)
    const prevR = 14 + Math.cos(reticlePulsePhase * 1.5) * 3;
    radarCtx.strokeStyle = '#f59e0b';
    radarCtx.lineWidth = 2;
    radarCtx.setLineDash([4, 3]);
    radarCtx.beginPath();
    radarCtx.arc(pgx, pgy, prevR, 0, 2 * Math.PI);
    radarCtx.stroke();

    radarCtx.beginPath();
    radarCtx.moveTo(pgx - prevR - 4, pgy); radarCtx.lineTo(pgx + prevR + 4, pgy);
    radarCtx.moveTo(pgx, pgy - prevR - 4); radarCtx.lineTo(pgx, pgy + prevR + 4);
    radarCtx.stroke();
    radarCtx.setLineDash([]);

    radarCtx.fillStyle = '#fbbf24';
    radarCtx.font = 'bold 10px "JetBrains Mono", monospace';
    radarCtx.fillText(`🎯 PENDING TARGET (${previewGoal.x.toFixed(1)}, ${previewGoal.y.toFixed(1)})`, pgx - 36, pgy + prevR + 13);

    // Dashed connector line from UGV to pending goal
    radarCtx.strokeStyle = '#f59e0b';
    radarCtx.lineWidth = 2;
    radarCtx.setLineDash([6, 5]);
    radarCtx.beginPath();
    radarCtx.moveTo(rx, ry);
    radarCtx.lineTo(pgx, pgy);
    radarCtx.stroke();
    radarCtx.setLineDash([]);
  }

  // H. Global Theta* Path (Cyan Dashed Line)
  const gpath = latestTelemetry.global_path || [];
  if (gpath.length > 1) {
    radarCtx.strokeStyle = '#38bdf8';
    radarCtx.lineWidth = 2;
    radarCtx.setLineDash([5, 4]);
    radarCtx.beginPath();
    const [p0x, p0y] = toCanvas(gpath[0][0], gpath[0][1]);
    radarCtx.moveTo(p0x, p0y);
    for (let i = 1; i < gpath.length; i++) {
      const [px, py] = toCanvas(gpath[i][0], gpath[i][1]);
      radarCtx.lineTo(px, py);
    }
    radarCtx.stroke();
    radarCtx.setLineDash([]);
  }

  // I. DWA Rollout Trajectory (Green)
  const dwaTraj = latestTelemetry.dwa_best_traj || [];
  if (dwaTraj.length > 1) {
    const rad = (pose.theta_deg * Math.PI) / 180;
    radarCtx.strokeStyle = '#10b981';
    radarCtx.lineWidth = 3;
    radarCtx.beginPath();
    radarCtx.moveTo(rx, ry);
    for (const pt of dwaTraj) {
      const wx = pose.x + pt[0] * Math.cos(rad) - pt[1] * Math.sin(rad);
      const wy = pose.y + pt[0] * Math.sin(rad) + pt[1] * Math.cos(rad);
      const [dx, dy] = toCanvas(wx, wy);
      radarCtx.lineTo(dx, dy);
    }
    radarCtx.stroke();
  }

  // J. Estimated Historical Trajectory (Amber Trail)
  const estTraj = latestTelemetry.estimated_trajectory || [];
  if (estTraj.length > 1) {
    radarCtx.strokeStyle = '#f59e0b';
    radarCtx.lineWidth = 1.5;
    radarCtx.beginPath();
    const [e0x, e0y] = toCanvas(estTraj[0][0], estTraj[0][1]);
    radarCtx.moveTo(e0x, e0y);
    for (let i = 1; i < estTraj.length; i++) {
      const [ex, ey] = toCanvas(estTraj[i][0], estTraj[i][1]);
      radarCtx.lineTo(ex, ey);
    }
    radarCtx.stroke();
  }

  // K. Forward 70° RGB-D Vision Sensor FOV Cone
  const rad = (pose.theta_deg * Math.PI) / 180;
  const fovHalfAngle = (35.0 * Math.PI) / 180; // 35 deg half-angle (70 deg total)
  const fovRangePx = Math.min(180, scale * 5.5);

  const fovGrad = radarCtx.createRadialGradient(rx, ry, 2, rx, ry, fovRangePx);
  fovGrad.addColorStop(0, 'rgba(6, 182, 212, 0.22)');
  fovGrad.addColorStop(0.75, 'rgba(6, 182, 212, 0.08)');
  fovGrad.addColorStop(1, 'rgba(6, 182, 212, 0.0)');

  radarCtx.fillStyle = fovGrad;
  radarCtx.beginPath();
  radarCtx.moveTo(rx, ry);
  radarCtx.arc(rx, ry, fovRangePx, -rad - fovHalfAngle, -rad + fovHalfAngle);
  radarCtx.closePath();
  radarCtx.fill();

  // FOV Peripheral Ray Lines
  radarCtx.strokeStyle = 'rgba(6, 182, 212, 0.35)';
  radarCtx.lineWidth = 1;
  radarCtx.beginPath();
  radarCtx.moveTo(rx, ry);
  radarCtx.lineTo(rx + fovRangePx * Math.cos(-rad - fovHalfAngle), ry + fovRangePx * Math.sin(-rad - fovHalfAngle));
  radarCtx.moveTo(rx, ry);
  radarCtx.lineTo(rx + fovRangePx * Math.cos(-rad + fovHalfAngle), ry + fovRangePx * Math.sin(-rad + fovHalfAngle));
  radarCtx.stroke();

  // L. UGV Robot Marker (Position, Heading & Chassis Footprint)
  radarCtx.save();
  radarCtx.translate(rx, ry);
  radarCtx.rotate(-rad); // Canvas rotates clockwise

  // Chassis Body (0.52m x 0.44m)
  const cw = Math.max(8, scale * 0.46);
  const cl = Math.max(10, scale * 0.54);
  radarCtx.fillStyle = '#0f172a';
  radarCtx.strokeStyle = '#38bdf8';
  radarCtx.lineWidth = 1.5;
  radarCtx.beginPath();
  radarCtx.rect(-cl / 2, -cw / 2, cl, cw);
  radarCtx.fill();
  radarCtx.stroke();

  // Wheels (Left and Right treads)
  const tw = Math.max(2, scale * 0.12);
  const tl = Math.max(4, scale * 0.28);
  radarCtx.fillStyle = '#334155';
  radarCtx.fillRect(-tl / 2, -cw / 2 - tw + 1, tl, tw);
  radarCtx.fillRect(-tl / 2, cw / 2 - 1, tl, tw);

  // Forward heading indicator
  radarCtx.fillStyle = '#f59e0b';
  radarCtx.beginPath();
  radarCtx.arc(cl / 3, 0, 3, 0, 2 * Math.PI);
  radarCtx.fill();

  radarCtx.restore();

  // M. 360° Rotating Radar Sweep Beam
  radarSweepAngle += 0.045;
  const sweepLen = Math.max(90, scale * 9);
  const sx = rx + sweepLen * Math.cos(radarSweepAngle);
  const sy = ry + sweepLen * Math.sin(radarSweepAngle);

  const sweepGrad = radarCtx.createLinearGradient(rx, ry, sx, sy);
  sweepGrad.addColorStop(0, 'rgba(52, 211, 153, 0.85)');
  sweepGrad.addColorStop(1, 'rgba(52, 211, 153, 0.0)');
  radarCtx.strokeStyle = sweepGrad;
  radarCtx.lineWidth = 2;
  radarCtx.beginPath();
  radarCtx.moveTo(rx, ry);
  radarCtx.lineTo(sx, sy);
  radarCtx.stroke();

  radarCtx.fillStyle = 'rgba(52, 211, 153, 0.04)';
  radarCtx.beginPath();
  radarCtx.moveTo(rx, ry);
  radarCtx.arc(rx, ry, sweepLen, radarSweepAngle - 0.35, radarSweepAngle);
  radarCtx.closePath();
  radarCtx.fill();

  // N. Avionics Digital Compass Tape Header (Top of Radar)
  const tapeW = 200;
  const tapeH = 22;
  const tapeX = w / 2 - tapeW / 2;
  const tapeY = 6;

  radarCtx.fillStyle = 'rgba(6, 10, 18, 0.85)';
  radarCtx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
  radarCtx.lineWidth = 1;
  radarCtx.beginPath();
  radarCtx.rect(tapeX, tapeY, tapeW, tapeH);
  radarCtx.fill();
  radarCtx.stroke();

  const normDeg = ((pose.theta_deg % 360) + 360) % 360;
  const cardinals = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  const cardDir = cardinals[Math.round(normDeg / 45) % 8];

  radarCtx.fillStyle = '#38bdf8';
  radarCtx.font = 'bold 10px "JetBrains Mono", monospace';
  radarCtx.textAlign = 'center';
  radarCtx.fillText(`HDG ${normDeg.toFixed(1)}° [${cardDir}]`, w / 2, tapeY + 15);
  radarCtx.textAlign = 'left'; // Reset

  // Center indicator notch
  radarCtx.fillStyle = '#f59e0b';
  radarCtx.beginPath();
  radarCtx.moveTo(w / 2 - 4, tapeY + tapeH);
  radarCtx.lineTo(w / 2 + 4, tapeY + tapeH);
  radarCtx.lineTo(w / 2, tapeY + tapeH - 4);
  radarCtx.closePath();
  radarCtx.fill();

  requestAnimationFrame(renderRadarFrame);
}
requestAnimationFrame(renderRadarFrame);

// 10. Tactical Actions (Inject Obstacle & Reset)
async function sendCmd(action) {
  try {
    await fetch('/api/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action: action })
    });
  } catch (e) {
    console.error(e);
  }
}

function triggerObstacleInjection() {
  const btn = document.getElementById('btn-inject-obstacle');
  if (btn) {
    btn.innerText = '⚠️ SUDDEN OBSTACLE INJECTED!';
    btn.style.backgroundColor = '#dc2626';
    btn.style.boxShadow = '0 0 18px rgba(239, 68, 68, 0.8)';
    setTimeout(() => {
      btn.innerText = '⚠️ INJECT SUDDEN OBSTACLE (LIVE DEMO)';
      btn.style.backgroundColor = '';
      btn.style.boxShadow = '';
    }, 3000);
  }
  addLogEntry(`[${(latestTelemetry.time || 0).toFixed(1)}s] ⚠️ USER TRIGGERED: Dynamic hazard dropped 2.8m directly ahead of UGV`, 'log-hazard');
  sendCmd('inject_obstacle');
}
