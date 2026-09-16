(function () {
  "use strict";
  const data = JSON.parse(document.getElementById("demo-data").textContent);
  const setText = (id, text) => {
    const el = document.getElementById(id);
    if (el) { el.textContent = text; }
  };
  if (!data || !data.frames || !data.frames.length) {
    setText("caption", "No demonstration data was embedded in this page.");
    return;
  }

  let frames = data.experiments[0].frames;
  const NS = "http://www.w3.org/2000/svg";
  const stage = document.getElementById("stage");
  stage.setAttribute("viewBox", "0 0 " + data.viewportPixels + " " + data.viewportPixels);
  const backLayer = document.createElementNS(NS, "g");
  const frontLayer = document.createElementNS(NS, "g");
  stage.appendChild(backLayer);
  stage.appendChild(frontLayer);

  const decimals = data.metricsDecimals;
  const fixed = (value) => Number(value).toFixed(decimals);
  const polyline = (classes, layer) => {
    const element = document.createElementNS(NS, "polyline");
    element.setAttribute("class", classes);
    element.setAttribute("fill", "none");
    element.setAttribute("stroke-linejoin", "round");
    element.setAttribute("stroke-linecap", "round");
    layer.appendChild(element);
    return element;
  };

  // One entry per shape of the first frame. Every frame has the same shapes in
  // the same order (guaranteed by the exporter), so the elements are built once.
  // Rings are split in two: a dim full ring at low opacity plus a bright arc for
  // the half that passes in front, which gives the flat SVG a sense of depth.
  const shapeElements = frames[0].shapes.map((shape) => ({
    kind: shape.kind,
    closed: shape.closed,
    main: polyline(
      "k-" + shape.kind + (shape.closed ? " ring dim" : " line"),
      shape.closed ? backLayer : frontLayer
    ),
    near: shape.closed ? polyline("k-" + shape.kind + " ring near", frontLayer) : null
  }));
  const axisElements = {};
  shapeElements.forEach((entry) => {
    if (entry.kind === "yawAxis" || entry.kind === "pitchAxis" || entry.kind === "rollAxis") {
      axisElements[entry.kind] = entry.main;
    }
  });
  const labelElements = frames[0].labels.map(() => {
    const element = document.createElementNS(NS, "text");
    element.setAttribute("class", "axis-label");
    frontLayer.appendChild(element);
    return element;
  });

  // Closed shapes repeat their first point so the polyline joins up.
  const toPoints = (points, closed) => {
    const list = closed && points.length ? points.concat([points[0]]) : points;
    return list.map((point) => point[0] + "," + point[1]).join(" ");
  };

  // The contiguous run of points facing the camera. It is drawn brighter on top
  // of the dim full ring, so each ring reads as a solid object rather than a flat
  // outline.
  const frontArc = (points) => {
    const count = points.length;
    let start = -1;
    for (let index = 0; index < count; index++) {
      if (points[index][2] >= 0 && points[(index - 1 + count) % count][2] < 0) {
        start = index;
        break;
      }
    }
    if (start < 0) {
      return points[0][2] >= 0 ? points : [];
    }
    const arc = [];
    for (let step = 0; step < count; step++) {
      const point = points[(start + step) % count];
      if (point[2] < 0) { break; }
      arc.push(point);
    }
    return arc;
  };

  const axisClass = (kind, state) => "k-" + kind + " line" + state;

  function drawFrame(index) {
    const frame = frames[index];
    frame.shapes.forEach((shape, position) => {
      const element = shapeElements[position];
      if (!element) { return; }
      element.main.setAttribute("points", toPoints(shape.points, shape.closed));
      if (element.near) {
        element.near.setAttribute("points", toPoints(frontArc(shape.points), false));
      }
    });

    // Highlight the two axes that are merging into each other.
    const gap = frame.metrics.axisAlignmentDegrees;
    const state = gap <= data.axisToleranceDegrees
      ? " aligned"
      : (gap <= data.axisWarningDegrees ? " converging" : "");
    if (axisElements.yawAxis) {
      axisElements.yawAxis.setAttribute("class", axisClass("yawAxis", state));
    }
    if (axisElements.rollAxis) {
      axisElements.rollAxis.setAttribute("class", axisClass("rollAxis", state));
    }

    frame.labels.forEach((label, position) => {
      const element = labelElements[position];
      if (!element) { return; }
      element.setAttribute("x", label.x);
      element.setAttribute("y", label.y);
      element.textContent = label.text;
    });
  }

  function updatePanel(frame) {
    const metrics = frame.metrics;
    setText("rd-roll", fixed(metrics.eulerDegrees[0]) + "°");
    setText("rd-pitch", fixed(metrics.eulerDegrees[1]) + "°");
    setText("rd-yaw", fixed(metrics.eulerDegrees[2]) + "°");
    setText("rd-quaternion", metrics.quaternion.map(fixed).join(", "));
    setText("rd-axis", fixed(metrics.axisAlignmentDegrees) + "°");
    setText("rd-safety", fixed(metrics.safetyFactor));
    setText("rd-lock", metrics.gimbalLock ? "yes" : "no");
    setText("rd-dof", String(metrics.degreesOfFreedomLost));
    setText("rd-singularity", metrics.singularity);

    const rows = [0, 1, 2].map((row) =>
      metrics.rotationMatrix.slice(row * 3, row * 3 + 3).map(fixed).join("  ")
    );
    setText("rd-matrix", rows.join("   |   "));

    const pill = document.getElementById("status-pill");
    if (pill) {
      pill.textContent = frame.status;
      pill.className = "pill " + frame.status;
    }
    setText("status-text", frame.label);
    setText("frame-label", frame.label);
    setText("caption", frame.explanation);
  }

  // Static content that never changes from frame to frame.
  setText("title", data.title);
  setText("subtitle", data.subtitle);

  const singularityPitch = data.equivalence.length
    ? fixed(data.equivalence[0].pitchDegrees) + "°"
    : "the singularity";
  setText(
    "equiv-intro",
    "Every row sits at pitch = " + singularityPitch + ". At that pitch the attitude depends " +
    "only on (yaw - roll), so rows sharing that difference describe exactly the same orientation " +
    "even though their angles differ wildly."
  );

  const equivalenceBody = document.getElementById("equiv-body");
  data.equivalence.forEach((row) => {
    const tableRow = document.createElement("tr");
    const cells = [
      fixed(row.rollDegrees) + "°",
      fixed(row.pitchDegrees) + "°",
      fixed(row.yawDegrees) + "°",
      fixed(row.invariantDegrees) + "°",
      row.quaternion.map(fixed).join(", "),
      row.matchesReference ? "same" : "different"
    ];
    cells.forEach((text, position) => {
      const cell = document.createElement("td");
      cell.textContent = text;
      if (position === 5) {
        cell.className = row.matchesReference ? "yes" : "no";
      }
      tableRow.appendChild(cell);
    });
    equivalenceBody.appendChild(tableRow);
  });

  const notes = document.getElementById("notes");
  data.notes.forEach((note) => {
    const item = document.createElement("li");
    item.textContent = note;
    notes.appendChild(item);
  });

  const slider = document.getElementById("slider");
  const playButton = document.getElementById("play");
  slider.max = String(frames.length - 1);

  let current = 0;
  let direction = 1;
  let timer = null;

  function show(index) {
    current = Math.max(0, Math.min(frames.length - 1, index));
    slider.value = String(current);
    drawFrame(current);
    updatePanel(frames[current]);
  }

  function stop() {
    if (timer !== null) {
      clearInterval(timer);
      timer = null;
    }
    playButton.textContent = "Play";
  }

  function start() {
    stop();
    // Ping-pong playback: sweep into the singularity, then back out again.
    timer = setInterval(() => {
      let next = current + direction;
      if (next >= frames.length || next < 0) {
        direction = -direction;
        next = current + direction;
      }
      show(next);
    }, 110);
    playButton.textContent = "Pause";
  }

  playButton.addEventListener("click", () => {
    if (timer === null) { start(); } else { stop(); }
  });
  slider.addEventListener("input", () => {
    stop();
    show(Number(slider.value));
  });
  document.addEventListener("keydown", (event) => {
    if (event.target.matches("input, select, button, a")) { return; }
    if (event.key === " ") {
      event.preventDefault();
      if (timer === null) { start(); } else { stop(); }
    } else if (event.key === "ArrowRight") {
      stop();
      show(current + 1);
    } else if (event.key === "ArrowLeft") {
      stop();
      show(current - 1);
    }
  });

  const experiment = document.getElementById("experiment");
  data.experiments.forEach((item, index) => {
    const option = document.createElement("option");
    option.value = String(index);
    option.textContent = item.title;
    experiment.appendChild(option);
  });
  function chooseExperiment() {
    stop();
    direction = 1;
    const selected = data.experiments[Number(experiment.value)];
    frames = selected.frames;
    slider.max = String(frames.length - 1);
    slider.setAttribute("aria-label", selected.controlLabel);
    setText("control-label", selected.controlLabel);
    setText("experiment-explanation", selected.explanation);
    show(0);
  }
  experiment.addEventListener("change", chooseExperiment);
  chooseExperiment();

  const qStage = document.getElementById("quaternion-stage");
  qStage.setAttribute("viewBox", "0 0 " + data.viewportPixels + " " + data.viewportPixels);
  const qShapes = data.quaternionFrames[0].shapes.map((shape) =>
    polyline("k-" + shape.kind + " line", qStage));
  const qSlider = document.getElementById("quaternion-slider");
  const qPlay = document.getElementById("quaternion-play");
  let qTimer = null;
  let qDirection = 1;
  function showQuaternion() {
    const frame = data.quaternionFrames[Number(qSlider.value)];
    frame.shapes.forEach((shape, index) => qShapes[index].setAttribute("points", toPoints(shape.points, shape.closed)));
    setText("quaternion-angle", frame.angleDegrees + "° about Y");
    setText("quaternion-value", "q (x, y, z, w) = (" + frame.quaternion.map(fixed).join(", ") + ")");
    setText("quaternion-caption", frame.angleDegrees === 90
      ? "At 90°: Euler roll/yaw controls would align here. The quaternion remains a valid orientation."
      : (frame.angleDegrees < 90 ? "Approaching the vertical pose." : "Past the vertical pose: rotation continues smoothly."));
  }
  function stopQuaternion() {
    clearInterval(qTimer);
    qTimer = null;
    qPlay.textContent = "Play quaternion motion";
  }
  qSlider.addEventListener("input", () => { stopQuaternion(); showQuaternion(); });
  qPlay.addEventListener("click", () => {
    if (qTimer !== null) { stopQuaternion(); return; }
    stop();
    qPlay.textContent = "Pause quaternion motion";
    qTimer = setInterval(() => {
      let next = Number(qSlider.value) + qDirection;
      if (next > 60 || next < 0) { qDirection *= -1; next = Number(qSlider.value) + qDirection; }
      qSlider.value = String(next);
      showQuaternion();
    }, 80);
  });
  showQuaternion();
})();
