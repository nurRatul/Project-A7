/**
 * Live radar canvas renderer.
 * Receives vehicle_update events over Socket.IO and draws at ~15–20 FPS
 * without full-page redraws.
 */
(() => {
  "use strict";

  const canvas = document.getElementById("radar");
  const ctx = canvas.getContext("2d");

  // HUD elements
  const elPos  = document.getElementById("hud-pos");
  const elHdg  = document.getElementById("hud-hdg");
  const elPath = document.getElementById("hud-path");
  const elMode = document.getElementById("hud-mode");
  const elStat = document.getElementById("hud-status");
  const elZoom = document.getElementById("zoom");
  const elZoomVal = document.getElementById("zoom-val");

  // View state
  let scale = 8;            // px per metre
  let viewX = 0;            // camera centre in local metres
  let viewY = 0;
  let follow = true;        // auto-centre on vehicle
  let state = null;         // latest snapshot
  let dpr = window.devicePixelRatio || 1;

  // Colours (match CSS)
  const COL = {
    bg: "#0a0f0a",
    grid: "#0d3b0d",
    gridBright: "#1a5c1a",
    trail: "#00ff66",
    trailDim: "#00aa4488",
    vehicle: "#00ff88",
    glow: "rgba(0,255,136,0.25)",
    text: "#7aaf7a",
    obstacle: "#ff4444",
  };

  // ------------------------------------------------------------------
  // Resize
  // ------------------------------------------------------------------
  function resize() {
    dpr = window.devicePixelRatio || 1;
    canvas.width  = window.innerWidth  * dpr;
    canvas.height = window.innerHeight * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  window.addEventListener("resize", resize);
  resize();

  // ------------------------------------------------------------------
  // Coordinate helpers
  // ------------------------------------------------------------------
  function worldToScreen(wx, wy) {
    const cx = canvas.width  / dpr / 2;
    const cy = canvas.height / dpr / 2;
    return {
      x: cx + (wx - viewX) * scale,
      y: cy - (wy - viewY) * scale,   // +Y (North) points up
    };
  }

  // ------------------------------------------------------------------
  // Drawing
  // ------------------------------------------------------------------
  function drawGrid() {
    const w = canvas.width  / dpr;
    const h = canvas.height / dpr;
    const cx = w / 2;
    const cy = h / 2;

    // Major grid every 5 m, minor every 1 m
    const minor = scale;
    const major = scale * 5;

    ctx.save();
    ctx.strokeStyle = COL.grid;
    ctx.lineWidth = 1;

    // Vertical lines
    const startX = cx - viewX * scale;
    const startY = cy + viewY * scale;

    for (let x = startX % minor; x < w; x += minor) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let y = startY % minor; y < h; y += minor) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // Major
    ctx.strokeStyle = COL.gridBright;
    for (let x = startX % major; x < w; x += major) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }
    for (let y = startY % major; y < h; y += major) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    // Origin crosshair
    const origin = worldToScreen(0, 0);
    ctx.strokeStyle = COL.trailDim;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(origin.x - 12, origin.y);
    ctx.lineTo(origin.x + 12, origin.y);
    ctx.moveTo(origin.x, origin.y - 12);
    ctx.lineTo(origin.x, origin.y + 12);
    ctx.stroke();

    // Axis labels
    ctx.fillStyle = COL.text;
    ctx.font = "11px monospace";
    ctx.fillText("N", origin.x + 4, origin.y - 16);
    ctx.fillText("E", origin.x + 16, origin.y + 4);

    ctx.restore();
  }

  function drawPath(path) {
    if (!path || path.length < 2) return;
    ctx.save();
    ctx.strokeStyle = COL.trail;
    ctx.lineWidth = 2;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.globalAlpha = 0.85;
    ctx.beginPath();
    const first = worldToScreen(path[0].x, path[0].y);
    ctx.moveTo(first.x, first.y);
    for (let i = 1; i < path.length; i++) {
      const p = worldToScreen(path[i].x, path[i].y);
      ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
    ctx.restore();
  }

  function drawObstacles(obstacles) {
    if (!obstacles || !obstacles.length) return;
    ctx.save();
    for (const o of obstacles) {
      const p = worldToScreen(o.x, o.y);
      const r = (o.radius || 0.3) * scale;
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.fillStyle = COL.obstacle + "55";
      ctx.fill();
      ctx.strokeStyle = COL.obstacle;
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawVehicle(x, y, headingDeg) {
    const p = worldToScreen(x, y);
    // heading: 0 = North, clockwise positive → canvas angle
    // canvas 0 is East, CCW; we want 0 = North, CW
    const rad = ((headingDeg - 90) * Math.PI) / 180;

    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(rad);

    // Glow
    ctx.beginPath();
    ctx.arc(0, 0, 18, 0, Math.PI * 2);
    ctx.fillStyle = COL.glow;
    ctx.fill();

    // Body (rectangle pointing +X after rotation, i.e. forward)
    const L = 14, W = 8;
    ctx.fillStyle = COL.vehicle;
    ctx.strokeStyle = "#003322";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(L, 0);          // nose
    ctx.lineTo(-L * 0.6, -W);
    ctx.lineTo(-L * 0.6,  W);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Heading arrow tip already is the nose
    ctx.restore();
  }

  function render() {
    const w = canvas.width  / dpr;
    const h = canvas.height / dpr;
    ctx.fillStyle = COL.bg;
    ctx.fillRect(0, 0, w, h);

    drawGrid();

    if (state) {
      if (follow && state.has_fix) {
        viewX = state.x;
        viewY = state.y;
      }
      drawPath(state.path);
      drawObstacles(state.obstacles);
      if (state.has_fix) {
        drawVehicle(state.x, state.y, state.heading);
      }
    }
  }

  // Fixed render loop (~20 FPS)
  setInterval(render, 50);

  // ------------------------------------------------------------------
  // HUD
  // ------------------------------------------------------------------
  function updateHud(s) {
    if (!s) return;
    elPos.textContent = s.has_fix
      ? `${s.x.toFixed(2)} m E, ${s.y.toFixed(2)} m N`
      : "no fix";
    elHdg.textContent = `${s.heading.toFixed(1)}°`;
    elPath.textContent = s.path_count;
    elMode.textContent = (s.meta && s.meta.mode) || "—";
    elStat.textContent = (s.meta && s.meta.message) || "";
  }

  // ------------------------------------------------------------------
  // Socket.IO
  // ------------------------------------------------------------------
  const socket = io({ transports: ["websocket", "polling"] });

  socket.on("connect", () => {
    elStat.textContent = "Connected";
  });
  socket.on("disconnect", () => {
    elStat.textContent = "Disconnected";
  });
  socket.on("vehicle_update", (data) => {
    state = data;
    if (data.scale_px_per_m && data.scale_px_per_m !== scale) {
      // server may advertise scale; keep local zoom control authoritative
    }
    updateHud(data);
  });

  // ------------------------------------------------------------------
  // Toolbar
  // ------------------------------------------------------------------
  document.getElementById("btn-center").addEventListener("click", () => {
    follow = true;
    if (state && state.has_fix) {
      viewX = state.x;
      viewY = state.y;
    }
  });

  document.getElementById("btn-reset").addEventListener("click", () => {
    socket.emit("reset_path");
    fetch("/api/vehicle/reset", { method: "POST" }).catch(() => {});
  });

  elZoom.addEventListener("input", () => {
    scale = Number(elZoom.value);
    elZoomVal.textContent = `${scale} px/m`;
  });

  // Pan with drag, disable follow while panning
  let dragging = false;
  let lastMX = 0, lastMY = 0;
  canvas.addEventListener("pointerdown", (e) => {
    dragging = true;
    follow = false;
    lastMX = e.clientX;
    lastMY = e.clientY;
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    const dx = e.clientX - lastMX;
    const dy = e.clientY - lastMY;
    viewX -= dx / scale;
    viewY += dy / scale;
    lastMX = e.clientX;
    lastMY = e.clientY;
  });
  canvas.addEventListener("pointerup", () => { dragging = false; });
  canvas.addEventListener("pointercancel", () => { dragging = false; });

  // Wheel zoom
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    const next = Math.max(2, Math.min(40, scale + (e.deltaY > 0 ? -1 : 1)));
    scale = next;
    elZoom.value = next;
    elZoomVal.textContent = `${next} px/m`;
  }, { passive: false });
})();
