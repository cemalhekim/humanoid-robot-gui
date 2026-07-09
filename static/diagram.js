// Self-contained .drawio (mxGraph XML) viewer for the dashboard "Diagrams" page.
// Renders the uncompressed mxGraph subset used by this repo's docs to SVG, with
// wheel-zoom-to-cursor, drag-to-pan, and fit/zoom controls. No external deps.
(function () {
  "use strict";

  const SVGNS = "http://www.w3.org/2000/svg";
  const XHTMLNS = "http://www.w3.org/1999/xhtml";
  const MIN_VIEW = 20; // smallest viewBox width/height (max zoom in)
  const MAX_VIEW_SCALE = 8; // max zoom out relative to fit

  const canvas = document.getElementById("diagramCanvas");
  if (!canvas) return;

  const fileSelect = document.getElementById("diagramFileSelect");
  const pageSelect = document.getElementById("diagramPageSelect");
  const emptyEl = document.getElementById("diagramEmpty");
  const zoomLabel = document.getElementById("diagramZoomLabel");
  const btnRefresh = document.getElementById("diagramRefresh");
  const btnZoomIn = document.getElementById("diagramZoomIn");
  const btnZoomOut = document.getElementById("diagramZoomOut");
  const btnFit = document.getElementById("diagramFit");

  const state = {
    doc: null, // { pages: [{name, compressed, cells}] }
    loadedFile: null,
    svg: null,
    baseView: null, // fit viewBox {x,y,w,h}
    view: null, // current viewBox
    listLoaded: false,
    initialLoaded: false,
  };

  // ---------- small helpers ----------
  const num = (v) => {
    const n = parseFloat(v);
    return Number.isFinite(n) ? n : 0;
  };
  const escapeHtml = (s) =>
    String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");

  function setEmpty(message) {
    if (!emptyEl) return;
    if (message) {
      emptyEl.textContent = message;
      emptyEl.style.display = "flex";
    } else {
      emptyEl.style.display = "none";
    }
  }

  // ---------- parsing ----------
  function parseStyle(styleStr) {
    const out = {};
    for (const token of String(styleStr || "").split(";")) {
      if (!token) continue;
      const eq = token.indexOf("=");
      if (eq === -1) out[token.trim()] = true;
      else out[token.slice(0, eq).trim()] = token.slice(eq + 1).trim();
    }
    return out;
  }

  function readCell(cell, labelOverride, idOverride) {
    const geom = cell.querySelector(":scope > mxGeometry");
    let geo = null;
    const points = [];
    let sourcePoint = null;
    let targetPoint = null;
    if (geom) {
      geo = {
        x: num(geom.getAttribute("x")),
        y: num(geom.getAttribute("y")),
        w: num(geom.getAttribute("width")),
        h: num(geom.getAttribute("height")),
        relative: geom.getAttribute("relative") === "1",
      };
      geom.querySelectorAll('Array[as="points"] > mxPoint').forEach((p) =>
        points.push({ x: num(p.getAttribute("x")), y: num(p.getAttribute("y")) }),
      );
      const sp = geom.querySelector('mxPoint[as="sourcePoint"]');
      const tp = geom.querySelector('mxPoint[as="targetPoint"]');
      if (sp) sourcePoint = { x: num(sp.getAttribute("x")), y: num(sp.getAttribute("y")) };
      if (tp) targetPoint = { x: num(tp.getAttribute("x")), y: num(tp.getAttribute("y")) };
    }
    return {
      id: idOverride || cell.getAttribute("id"),
      value: labelOverride != null ? labelOverride : cell.getAttribute("value") || "",
      style: parseStyle(cell.getAttribute("style")),
      vertex: cell.getAttribute("vertex") === "1",
      edge: cell.getAttribute("edge") === "1",
      parent: cell.getAttribute("parent"),
      source: cell.getAttribute("source"),
      target: cell.getAttribute("target"),
      geo,
      points,
      sourcePoint,
      targetPoint,
    };
  }

  function parseCells(model) {
    const root = model.querySelector("root") || model;
    const cells = [];
    for (const node of Array.from(root.children)) {
      const name = node.nodeName;
      if (name === "mxCell") {
        cells.push(readCell(node, null, null));
      } else if (name === "object" || name === "UserObject") {
        const inner = node.querySelector(":scope > mxCell");
        if (inner) {
          const label = node.getAttribute("label");
          cells.push(readCell(inner, label != null ? label : "", node.getAttribute("id")));
        }
      }
    }
    return cells;
  }

  function parseMxfile(text) {
    const doc = new DOMParser().parseFromString(text, "application/xml");
    if (doc.querySelector("parsererror")) throw new Error("Invalid diagram XML");
    const diagrams = Array.from(doc.querySelectorAll("diagram"));
    const pages = [];
    if (diagrams.length) {
      for (const d of diagrams) {
        const model = d.querySelector("mxGraphModel");
        if (model) {
          pages.push({ name: d.getAttribute("name") || "", compressed: false, cells: parseCells(model) });
        } else {
          // Content stored deflate+base64 (draw.io "compressed"). Not supported here.
          pages.push({ name: d.getAttribute("name") || "", compressed: true, cells: [] });
        }
      }
    } else {
      const model = doc.querySelector("mxGraphModel");
      if (model) pages.push({ name: "", compressed: false, cells: parseCells(model) });
    }
    return { pages };
  }

  // ---------- geometry ----------
  function makeIndex(cells) {
    const byId = new Map();
    for (const c of cells) byId.set(c.id, c);
    return byId;
  }

  function absPos(cell, byId) {
    let x = cell.geo ? cell.geo.x : 0;
    let y = cell.geo ? cell.geo.y : 0;
    let p = cell.parent ? byId.get(cell.parent) : null;
    let guard = 0;
    while (p && p.vertex && p.geo && guard++ < 50) {
      x += p.geo.x;
      y += p.geo.y;
      p = p.parent ? byId.get(p.parent) : null;
    }
    return { x, y };
  }

  function rectOf(cell, byId) {
    const { x, y } = absPos(cell, byId);
    return { x, y, w: cell.geo.w, h: cell.geo.h };
  }
  const centerOf = (r) => ({ x: r.x + r.w / 2, y: r.y + r.h / 2 });

  function borderPoint(r, toward) {
    const cx = r.x + r.w / 2;
    const cy = r.y + r.h / 2;
    let dx = toward.x - cx;
    let dy = toward.y - cy;
    if (dx === 0 && dy === 0) return { x: cx, y: cy };
    const hw = r.w / 2 || 1;
    const hh = r.h / 2 || 1;
    const scale = 1 / Math.max(Math.abs(dx) / hw, Math.abs(dy) / hh);
    return { x: cx + dx * scale, y: cy + dy * scale };
  }

  function orthogonalWaypoints(a, b) {
    if (Math.abs(b.x - a.x) < 1 || Math.abs(b.y - a.y) < 1) return [];
    if (Math.abs(b.y - a.y) >= Math.abs(b.x - a.x)) {
      const midY = (a.y + b.y) / 2;
      return [
        { x: a.x, y: midY },
        { x: b.x, y: midY },
      ];
    }
    const midX = (a.x + b.x) / 2;
    return [
      { x: midX, y: a.y },
      { x: midX, y: b.y },
    ];
  }

  // ---------- SVG element builders ----------
  const svgEl = (name, attrs) => {
    const el = document.createElementNS(SVGNS, name);
    if (attrs) for (const k in attrs) el.setAttribute(k, attrs[k]);
    return el;
  };

  const colorOf = (v, fallback) => (v === "none" ? "none" : v || fallback);

  function shapeKind(style) {
    if (style.ellipse) return "ellipse";
    if (style.rhombus) return "rhombus";
    if (style.shape === "note") return "note";
    if (style.triangle) return "triangle";
    if (style.text || style.strokeColor === "none") return "text";
    return "rect";
  }

  function labelElement(cell, x, y, w, h) {
    const raw = cell.value;
    if (!raw) return null;
    const tmp = String(raw).replace(/<[^>]+>/g, "").replace(/&[a-z#0-9]+;/gi, " ").trim();
    if (!tmp) return null;
    const s = cell.style;
    const fo = svgEl("foreignObject", { x, y, width: Math.max(1, w), height: Math.max(1, h) });
    const div = document.createElementNS(XHTMLNS, "div");
    div.setAttribute("class", "diagram-label");
    const align = s.align || "center";
    const valign = s.verticalAlign || "middle";
    div.style.justifyContent = align === "left" ? "flex-start" : align === "right" ? "flex-end" : "center";
    div.style.alignItems = valign === "top" ? "flex-start" : valign === "bottom" ? "flex-end" : "center";
    div.style.textAlign = align;
    div.style.whiteSpace = s.whiteSpace === "wrap" ? "normal" : "pre-wrap";
    if (s.fontSize) div.style.fontSize = num(s.fontSize) + "px";
    if (s.fontColor && s.fontColor !== "none") div.style.color = s.fontColor;
    const fontStyle = num(s.fontStyle);
    if (fontStyle & 1) div.style.fontWeight = "700";
    if (fontStyle & 2) div.style.fontStyle = "italic";
    if (fontStyle & 4) div.style.textDecoration = "underline";
    let html = String(raw).replace(/\n/g, "<br>");
    html = html.replace(/<\s*script[^>]*>[\s\S]*?<\/\s*script>/gi, "");
    div.innerHTML = html;
    fo.appendChild(div);
    return fo;
  }

  function renderVertex(cell, byId, group, bounds) {
    if (!cell.geo || cell.geo.w <= 0 || cell.geo.h <= 0) return;
    const { x, y } = absPos(cell, byId);
    const w = cell.geo.w;
    const h = cell.geo.h;
    const s = cell.style;
    const kind = shapeKind(s);
    const stroke = colorOf(s.strokeColor, kind === "text" ? "none" : "#000000");
    const fill = colorOf(s.fillColor, kind === "text" ? "none" : "#ffffff");
    const opacity = s.opacity != null ? Math.max(0, Math.min(1, num(s.opacity) / 100)) : 1;
    const common = {
      fill,
      stroke,
      "stroke-width": s.strokeWidth ? num(s.strokeWidth) : 1,
      opacity,
    };
    if (s.dashed === "1" || s.dashed === true) common["stroke-dasharray"] = "6 4";

    let shape = null;
    if (kind === "ellipse") {
      shape = svgEl("ellipse", { cx: x + w / 2, cy: y + h / 2, rx: w / 2, ry: h / 2, ...common });
    } else if (kind === "rhombus") {
      const pts = `${x + w / 2},${y} ${x + w},${y + h / 2} ${x + w / 2},${y + h} ${x},${y + h / 2}`;
      shape = svgEl("polygon", { points: pts, ...common });
    } else if (kind === "triangle") {
      const pts = `${x},${y} ${x + w},${y + h / 2} ${x},${y + h}`;
      shape = svgEl("polygon", { points: pts, ...common });
    } else if (kind === "note") {
      const f = Math.min(18, w * 0.35, h * 0.35);
      const d = `M ${x} ${y} L ${x + w - f} ${y} L ${x + w} ${y + f} L ${x + w} ${y + h} L ${x} ${y + h} Z`;
      shape = svgEl("path", { d, ...common });
    } else if (kind === "text") {
      shape = null; // label only
    } else {
      const attrs = { x, y, width: w, height: h, ...common };
      if (s.rounded === "1") {
        attrs.rx = 12;
        attrs.ry = 12;
      }
      shape = svgEl("rect", attrs);
    }
    if (shape) group.appendChild(shape);
    const label = labelElement(cell, x, y, w, h);
    if (label) group.appendChild(label);

    bounds.minX = Math.min(bounds.minX, x);
    bounds.minY = Math.min(bounds.minY, y);
    bounds.maxX = Math.max(bounds.maxX, x + w);
    bounds.maxY = Math.max(bounds.maxY, y + h);
  }

  function renderEdge(cell, byId, group, bounds) {
    const s = byId.get(cell.source);
    const t = byId.get(cell.target);
    const sRect = s && s.geo ? rectOf(s, byId) : null;
    const tRect = t && t.geo ? rectOf(t, byId) : null;
    let p0;
    let p1;
    if (sRect && tRect) {
      p0 = borderPoint(sRect, cell.points[0] || centerOf(tRect));
      p1 = borderPoint(tRect, cell.points[cell.points.length - 1] || centerOf(sRect));
    } else {
      p0 = cell.sourcePoint || (sRect ? centerOf(sRect) : null);
      p1 = cell.targetPoint || (tRect ? centerOf(tRect) : null);
    }
    if (!p0 || !p1) return;

    const waypoints = cell.points && cell.points.length ? cell.points : orthogonalWaypoints(p0, p1);
    const pts = [p0, ...waypoints, p1];
    const st = cell.style;
    const stroke = colorOf(st.strokeColor, "#33475b");
    const d = pts.map((p, i) => `${i === 0 ? "M" : "L"} ${p.x} ${p.y}`).join(" ");
    const path = svgEl("path", {
      d,
      fill: "none",
      stroke,
      "stroke-width": st.strokeWidth ? num(st.strokeWidth) : 1.4,
    });
    if (st.dashed === "1" || st.dashed === true) path.setAttribute("stroke-dasharray", "6 4");
    if (st.endArrow !== "none") path.setAttribute("marker-end", "url(#diagArrow)");
    group.appendChild(path);

    for (const p of pts) {
      bounds.minX = Math.min(bounds.minX, p.x);
      bounds.minY = Math.min(bounds.minY, p.y);
      bounds.maxX = Math.max(bounds.maxX, p.x);
      bounds.maxY = Math.max(bounds.maxY, p.y);
    }

    if (cell.value && String(cell.value).replace(/<[^>]+>/g, "").trim()) {
      const mid = pts[Math.floor((pts.length - 1) / 2)];
      const midNext = pts[Math.floor((pts.length - 1) / 2) + 1] || mid;
      const lx = (mid.x + midNext.x) / 2;
      const ly = (mid.y + midNext.y) / 2;
      const lines = String(cell.value).replace(/<br\s*\/?>/gi, "\n").replace(/<[^>]+>/g, "").split("\n");
      const fontColor = colorOf(st.fontColor, "#33475b");
      const text = svgEl("text", {
        x: lx,
        y: ly - (lines.length - 1) * 6,
        "text-anchor": "middle",
        "dominant-baseline": "middle",
        "font-size": st.fontSize ? num(st.fontSize) : 11,
        fill: fontColor,
        stroke: "#f4f5f7",
        "stroke-width": 3,
        "paint-order": "stroke",
        "font-family": "Helvetica Neue, Helvetica, Arial, sans-serif",
      });
      lines.forEach((line, i) => {
        const tspan = svgEl("tspan", { x: lx, dy: i === 0 ? 0 : 13 });
        tspan.textContent = line;
        text.appendChild(tspan);
      });
      group.appendChild(text);
    }
  }

  function buildSvg(cells) {
    const byId = makeIndex(cells);
    const svg = svgEl("svg", { xmlns: SVGNS });
    const defs = svgEl("defs", {});
    const marker = svgEl("marker", {
      id: "diagArrow",
      viewBox: "0 0 10 10",
      refX: "9",
      refY: "5",
      markerWidth: "7",
      markerHeight: "7",
      orient: "auto-start-reverse",
      markerUnits: "userSpaceOnUse",
    });
    const mpath = svgEl("path", { d: "M 0 0 L 10 5 L 0 10 z", fill: "context-stroke" });
    marker.appendChild(mpath);
    defs.appendChild(marker);
    svg.appendChild(defs);

    const edgeG = svgEl("g", {});
    const nodeG = svgEl("g", {});
    svg.appendChild(edgeG);
    svg.appendChild(nodeG);

    const bounds = { minX: Infinity, minY: Infinity, maxX: -Infinity, maxY: -Infinity };
    for (const cell of cells) {
      if (cell.edge) renderEdge(cell, byId, edgeG, bounds);
    }
    for (const cell of cells) {
      if (cell.vertex) renderVertex(cell, byId, nodeG, bounds);
    }

    if (!Number.isFinite(bounds.minX)) {
      bounds.minX = 0;
      bounds.minY = 0;
      bounds.maxX = 100;
      bounds.maxY = 100;
    }
    const pad = 40;
    const base = {
      x: bounds.minX - pad,
      y: bounds.minY - pad,
      w: bounds.maxX - bounds.minX + pad * 2,
      h: bounds.maxY - bounds.minY + pad * 2,
    };
    return { svg, base };
  }

  // ---------- view control ----------
  function applyView() {
    if (!state.svg || !state.view) return;
    const v = state.view;
    state.svg.setAttribute("viewBox", `${v.x} ${v.y} ${v.w} ${v.h}`);
    if (zoomLabel && state.baseView) {
      const pct = Math.round((state.baseView.w / v.w) * 100);
      zoomLabel.textContent = `${pct}%`;
    }
  }

  function fit() {
    if (!state.baseView) return;
    state.view = { ...state.baseView };
    applyView();
  }

  function clampView() {
    if (!state.baseView || !state.view) return;
    const maxW = state.baseView.w * MAX_VIEW_SCALE;
    const maxH = state.baseView.h * MAX_VIEW_SCALE;
    if (state.view.w > maxW) state.view.w = maxW;
    if (state.view.h > maxH) state.view.h = maxH;
    if (state.view.w < MIN_VIEW) state.view.w = MIN_VIEW;
    if (state.view.h < MIN_VIEW) state.view.h = MIN_VIEW;
  }

  function zoomAt(clientX, clientY, factor) {
    if (!state.svg || !state.view) return;
    const ctm = state.svg.getScreenCTM();
    let loc;
    if (ctm) {
      const pt = state.svg.createSVGPoint();
      pt.x = clientX;
      pt.y = clientY;
      loc = pt.matrixTransform(ctm.inverse());
    } else {
      loc = { x: state.view.x + state.view.w / 2, y: state.view.y + state.view.h / 2 };
    }
    const v = state.view;
    v.x = loc.x - (loc.x - v.x) * factor;
    v.y = loc.y - (loc.y - v.y) * factor;
    v.w *= factor;
    v.h *= factor;
    clampView();
    applyView();
  }

  function zoomCenter(factor) {
    const rect = canvas.getBoundingClientRect();
    zoomAt(rect.left + rect.width / 2, rect.top + rect.height / 2, factor);
  }

  // pan + wheel handlers
  const drag = { active: false, x: 0, y: 0, ax: 1, ad: 1, vx: 0, vy: 0 };

  function onWheel(e) {
    if (!state.svg) return;
    e.preventDefault();
    const factor = e.deltaY < 0 ? 0.88 : 1.136;
    zoomAt(e.clientX, e.clientY, factor);
  }

  function onPointerDown(e) {
    if (!state.svg || e.button !== 0) return;
    const ctm = state.svg.getScreenCTM();
    if (!ctm) return;
    drag.active = true;
    drag.x = e.clientX;
    drag.y = e.clientY;
    drag.ax = ctm.a || 1;
    drag.ad = ctm.d || 1;
    drag.vx = state.view.x;
    drag.vy = state.view.y;
    canvas.classList.add("is-dragging");
    canvas.setPointerCapture(e.pointerId);
  }

  function onPointerMove(e) {
    if (!drag.active || !state.view) return;
    state.view.x = drag.vx - (e.clientX - drag.x) / drag.ax;
    state.view.y = drag.vy - (e.clientY - drag.y) / drag.ad;
    applyView();
  }

  function onPointerUp(e) {
    if (!drag.active) return;
    drag.active = false;
    canvas.classList.remove("is-dragging");
    try {
      canvas.releasePointerCapture(e.pointerId);
    } catch (_) {
      /* ignore */
    }
  }

  // ---------- rendering pages ----------
  function mountSvg(svg, base) {
    if (state.svg && state.svg.parentNode) state.svg.parentNode.removeChild(state.svg);
    svg.setAttribute("preserveAspectRatio", "xMidYMid meet");
    canvas.appendChild(svg);
    state.svg = svg;
    state.baseView = base;
    fit();
  }

  function renderPage(index) {
    if (!state.doc) return;
    const page = state.doc.pages[index];
    if (!page) return;
    if (page.compressed) {
      if (state.svg && state.svg.parentNode) state.svg.parentNode.removeChild(state.svg);
      state.svg = null;
      setEmpty("This diagram page is stored compressed. In draw.io use Extras ▸ Edit Diagram / re-save uncompressed to view it here.");
      if (zoomLabel) zoomLabel.textContent = "--";
      return;
    }
    setEmpty(null);
    const { svg, base } = buildSvg(page.cells);
    mountSvg(svg, base);
  }

  async function loadFile(name) {
    if (!name) {
      setEmpty("No .drawio files found in docs/.");
      return;
    }
    setEmpty("Loading…");
    try {
      const res = await fetch("/api/diagrams/" + encodeURIComponent(name), { cache: "no-store" });
      if (!res.ok) throw new Error("HTTP " + res.status);
      const text = await res.text();
      state.doc = parseMxfile(text);
      state.loadedFile = name;
      const pages = state.doc.pages;
      if (pages.length > 1) {
        pageSelect.classList.remove("is-hidden");
        pageSelect.innerHTML = pages
          .map((p, i) => `<option value="${i}">${escapeHtml(p.name || "Page " + (i + 1))}</option>`)
          .join("");
        pageSelect.value = "0";
      } else {
        pageSelect.classList.add("is-hidden");
      }
      if (!pages.length) {
        setEmpty("This file has no diagram pages.");
        return;
      }
      renderPage(0);
    } catch (err) {
      setEmpty("Could not load diagram: " + (err && err.message ? err.message : err));
    }
  }

  async function loadFileList(autoLoad) {
    try {
      const res = await fetch("/api/diagrams", { cache: "no-store" });
      const data = await res.json();
      const files = data.files || [];
      const prev = fileSelect.value;
      fileSelect.innerHTML = files.length
        ? files.map((f) => `<option value="${escapeHtml(f.name)}">${escapeHtml(f.name)}</option>`).join("")
        : `<option value="">No .drawio files in docs/</option>`;
      if (prev && files.some((f) => f.name === prev)) fileSelect.value = prev;
      state.listLoaded = true;
      if (files.length && autoLoad && fileSelect.value !== state.loadedFile) {
        await loadFile(fileSelect.value);
      } else if (!files.length) {
        setEmpty("No .drawio files found in docs/.");
      }
    } catch (err) {
      setEmpty("Could not list diagrams: " + (err && err.message ? err.message : err));
    }
  }

  async function ensureLoaded() {
    if (!state.listLoaded) await loadFileList(true);
    else if (!state.initialLoaded && fileSelect.value) await loadFile(fileSelect.value);
    state.initialLoaded = true;
    // Re-fit once the panel is actually visible so aspect ratio is correct.
    if (state.svg) requestAnimationFrame(() => applyView());
  }

  // ---------- wiring ----------
  fileSelect.addEventListener("change", () => loadFile(fileSelect.value));
  pageSelect.addEventListener("change", () => renderPage(num(pageSelect.value)));
  btnRefresh && btnRefresh.addEventListener("click", () => loadFileList(false));
  btnZoomIn && btnZoomIn.addEventListener("click", () => zoomCenter(0.8));
  btnZoomOut && btnZoomOut.addEventListener("click", () => zoomCenter(1.25));
  btnFit && btnFit.addEventListener("click", fit);

  canvas.addEventListener("wheel", onWheel, { passive: false });
  canvas.addEventListener("pointerdown", onPointerDown);
  canvas.addEventListener("pointermove", onPointerMove);
  canvas.addEventListener("pointerup", onPointerUp);
  canvas.addEventListener("pointercancel", onPointerUp);
  canvas.addEventListener("dblclick", () => fit());

  window.addEventListener("hashchange", () => {
    if (window.location.hash === "#diagramPage") ensureLoaded();
  });
  window.addEventListener("resize", () => {
    if (state.svg) applyView();
  });

  if (window.location.hash === "#diagramPage") ensureLoaded();
  else loadFileList(false); // pre-populate the dropdown quietly
})();
