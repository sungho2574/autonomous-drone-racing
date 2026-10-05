"use strict";
const $ = (id) => document.getElementById(id),
  palette = [
    "#81c784",
    "#ff6b6b",
    "#4ba3ff",
    "#bd94ff",
    "#ffc471",
    "#61d7d1",
    "#ea91be",
    "#b2c2dd",
  ];
let runs = [],
  selected = null,
  plots = [],
  observers = [],
  request = 0,
  payload = null;
const el = (tag, text, cls) => {
  const n = document.createElement(tag);
  if (text !== undefined) n.textContent = text;
  if (cls) n.className = cls;
  return n;
};
function toast(text) {
  $("toast").textContent = text;
  $("toast").style.display = "block";
  setTimeout(() => ($("toast").style.display = "none"), 3500);
}
function setCollapsed(collapsed) {
  document.body.classList.toggle("collapsed", collapsed);
  $("collapse").setAttribute("aria-expanded", String(!collapsed));
  $("sidebar").inert = collapsed;
}
$("collapse").onclick = () =>
  setCollapsed(!document.body.classList.contains("collapsed"));
setCollapsed(innerWidth < 900);
function listRuns() {
  const q = $("search").value.toLowerCase();
  $("runs").replaceChildren();
  for (const r of runs) {
    if (!`${r.track} ${r.created_utc} ${r.id}`.toLowerCase().includes(q))
      continue;
    const b = el(
      "button",
      undefined,
      "run" + (r.id === selected ? " selected" : ""),
    );
    b.append(
      el("strong", r.track),
      el("small", new Date(r.created_utc).toLocaleString()),
      el("small", `${r.status} · ${r.parameters?.vio_profile || "default"}`),
    );
    b.onclick = () => selectRun(r.id);
    $("runs").append(b);
  }
  if (!runs.length)
    $("runs").append(el("p", "아직 저장된 비행이 없습니다.", "muted"));
}
async function refresh() {
  try {
    const r = await fetch("/api/runs");
    if (!r.ok) throw Error("목록을 읽지 못했습니다.");
    runs = await r.json();
    listRuns();
  } catch (e) {
    toast(e.message);
  }
}
$("refresh").onclick = refresh;
$("search").oninput = listRuns;
$("reload").onclick = () => selected && selectRun(selected);
function dispose() {
  observers.forEach((o) => o.disconnect());
  plots.forEach((p) => p.destroy());
  plots = [];
  observers = [];
  $("charts").replaceChildren();
}
async function selectRun(id) {
  selected = id;
  listRuns();
  const ticket = ++request;
  $("empty").hidden = false;
  $("empty").replaceChildren(
    el("h1", "기록 분석 중…"),
    el("p", "게이트 통과점과 시계열을 계산하고 있습니다.", "muted"),
  );
  $("report").hidden = true;
  dispose();
  try {
    const r = await fetch("/api/runs/" + encodeURIComponent(id));
    const d = await r.json();
    if (!r.ok) throw Error(d.error || "분석 실패");
    if (ticket !== request) return;
    payload = d;
    $("empty").hidden = true;
    $("report").hidden = false;
    $("title").textContent = d.meta.track;
    $("date").textContent =
      new Date(d.meta.created_utc).toLocaleString() + " · " + d.meta.id;
    $("summary").replaceChildren();
    for (const [label, value] of [
      ["기록 시간", d.duration.toFixed(1) + " s"],
      ["VIO 프로필", d.meta.parameters.vio_profile || "default"],
      ["계획 랩 수", d.meta.parameters.laps],
      ["종료 상태", d.meta.status],
    ]) {
      const s = el("div", undefined, "stat");
      s.append(el("small", label), el("strong", String(value)));
      $("summary").append(s);
    }
    $("warnings").replaceChildren(
      ...d.warnings.map((w) => el("div", w, "warning")),
    );
    $("gates").src = d.image_url + "?v=" + Date.now();
    $("download").href = d.image_url;
    $("metadata").textContent = JSON.stringify(
      { metadata: d.meta, controller_states: d.states },
      null,
      2,
    );
    renderCharts(d.series);
  } catch (e) {
    if (ticket !== request) return;
    $("empty").replaceChildren(
      el("h1", "기록을 표시할 수 없습니다."),
      el("p", e.message),
    );
  }
}
$("copy").onclick = async () => {
  try {
    const url = payload.image_url;
    if (!navigator.clipboard?.write || !window.ClipboardItem)
      throw Error(
        "이 브라우저는 이미지 복사를 지원하지 않습니다. PNG 저장을 사용하세요.",
      );
    const blob = fetch(url).then((r) => {
      if (!r.ok) throw Error("이미지 읽기 실패");
      return r.blob();
    });
    await navigator.clipboard.write([new ClipboardItem({ "image/png": blob })]);
    toast("전체 게이트 이미지를 복사했습니다.");
  } catch (e) {
    toast(e.message);
  }
};
const labels = {
  slam_features: "SLAM landmarks",
  msckf_update_features: "MSCKF update features",
  loop_triangulated_features: "Triangulated loop features",
  slam_capacity: "SLAM capacity",
  slam_utilization_pct: "SLAM capacity used",
  raw_error_sync_m: "Raw VIO ↔ actual",
  corrected_error_sync_m: "Corrected VIO ↔ actual",
};
const pretty = (k) => labels[k] || k.replaceAll("_", " ");
function aligned(keys, all) {
  const x = [...new Set(keys.flatMap((k) => all[k].map((p) => p[0])))].sort(
    (a, b) => a - b,
  );
  return [
    x,
    ...keys.map((k) => {
      const p = all[k];
      let j = 0;
      return x.map((t) => {
        while (j + 1 < p.length && p[j + 1][0] <= t) j++;
        if (p[j][0] === t) return p[j][1];
        if (
          j + 1 === p.length ||
          t < p[j][0] ||
          p[j][1] == null ||
          p[j + 1][1] == null
        )
          return null;
        const f = (t - p[j][0]) / (p[j + 1][0] - p[j][0]);
        return p[j][1] + f * (p[j + 1][1] - p[j][1]);
      });
    }),
  ];
}
function renderCharts(all) {
  const used = new Set();
  const groups = [
    [
      "특징점 수",
      "count",
      "SLAM은 필터 상태에 유지되는 점, MSCKF는 최근 업데이트 점입니다. 전체 KLT 추적점 수가 아닙니다.",
      [
        "slam_features",
        "msckf_update_features",
        "loop_triangulated_features",
        "slam_capacity",
      ],
    ],
    [
      "SLAM 용량 사용률",
      "%",
      "점이 많아도 재방문 인식이나 loop closure를 뜻하지 않습니다.",
      ["slam_utilization_pct"],
    ],
    [
      "위치 오차",
      "m",
      "동일 시각으로 보간한 실제 궤적과의 3D 거리. 긴 데이터 공백은 제외합니다.",
      ["raw_error_sync_m", "corrected_error_sync_m"],
    ],
    [
      "처리 시간",
      "ms",
      "OpenVINS 내부 단계별 소요 시간 · wall clock",
      Object.keys(all).filter((k) => k.startsWith("timing_")),
    ],
    [
      "수신 주파수",
      "Hz",
      "메시지 header stamp 기준",
      Object.keys(all).filter((k) => k.endsWith("_hz")),
    ],
    [
      "메시지 지연",
      "s",
      "수신 시 ROS 시간 − 메시지 시각",
      Object.keys(all).filter((k) => k.endsWith("_lag_s")),
    ],
    [
      "마지막 수신 이후",
      "s",
      "wall clock 기준; 시뮬레이션 일시정지에도 증가합니다.",
      Object.keys(all).filter((k) => k.endsWith("_age_s")),
    ],
    ...["x", "y", "z"].map((a) => [
      `${a.toUpperCase()} 위치`,
      "m",
      "map 좌표계",
      ["plan", "actual", "corrected", "raw"].map((n) => `${n}_${a}_m`),
    ]),
    [
      "PnP 누적 위치 보정",
      "m",
      "VIO에 적용되는 drift 보정량",
      ["drift_x_m", "drift_y_m", "drift_z_m"],
    ],
    [
      "속도",
      "m/s",
      "각 odometry가 제공한 속도",
      Object.keys(all).filter((k) => k.endsWith("_speed_mps")),
    ],
    [
      "Yaw",
      "deg",
      "±180° 경계에서 표시가 꺾일 수 있습니다.",
      Object.keys(all).filter((k) => k.endsWith("_yaw_deg")),
    ],
    [
      "VIO 위치 불확실성",
      "m",
      "필터 공분산의 표준편차 (1σ)",
      ["vio_sigma_x", "vio_sigma_y", "vio_sigma_z"],
    ],
    [
      "VIO 자세 불확실성",
      "rad",
      "필터 공분산의 표준편차 (1σ)",
      ["vio_sigma_roll", "vio_sigma_pitch", "vio_sigma_yaw"],
    ],
    [
      "게이트 검출",
      "count",
      "검출 수와 corner 유효 검출 수",
      ["gate_detections", "gate_valid_quads"],
    ],
    ["PnP 품질", "score", "게이트 PnP가 발행하는 품질 지표", ["pnp_quality"]],
    [
      "PnP 재투영 오차",
      "px",
      "급증 구간과 VIO 위치 점프를 함께 확인하세요.",
      ["pnp_reproj_px"],
    ],
    ["PnP 거리", "m", "게이트까지 추정 거리", ["pnp_distance_m"]],
    ["PnP 게이트 ID", "ID", "현재 관측을 연결한 게이트", ["pnp_gate_id"]],
    [
      "온라인 위치 오차",
      "m",
      "온라인 콜백 시점 비교; 위의 시간 정렬 오차와 다를 수 있습니다.",
      ["raw_error_online_m", "corrected_error_online_m"],
    ],
  ];
  for (const [title, unit, description, candidates] of groups) {
    const keys = candidates.filter((k) => all[k]?.length);
    keys.forEach((k) => used.add(k));
    if (keys.length) chart(title, unit, description, keys, all);
  }
  const extra = Object.keys(all).filter(
    (k) => !used.has(k) && !/^raw_vio_[xyz]_m$/.test(k),
  );
  if (extra.length)
    chart(
      "추가 진단 값",
      "",
      "서로 단위가 다른 항목은 개별 선택해서 확인하세요.",
      extra,
      all,
    );
  if (!plots.length)
    $("charts").append(el("p", "기록된 진단 값이 없습니다.", "muted"));
}
function chart(title, unit, description, keys, all) {
  const panel = el("section", undefined, "panel");
  panel.append(
    el("h2", title + (unit ? " · " + unit : "")),
    el("p", description, "chart-subtitle"),
  );
  const controls = el("div", undefined, "series-controls"),
    target = el("div", undefined, "plot");
  panel.append(controls, target);
  $("charts").append(panel);
  const values = [],
    inputs = [];
  keys.forEach((k, i) => {
    const label = el("label", undefined, "series-toggle");
    label.style.setProperty("--series-color", palette[i % palette.length]);
    const input = el("input");
    input.type = "checkbox";
    input.checked = true;
    input.setAttribute("aria-label", pretty(k));
    const value = el("span", "—", "series-value");
    label.append(
      input,
      el("span", undefined, "dot"),
      el("span", pretty(k)),
      value,
    );
    values.push(value);
    inputs.push(input);
    controls.append(label);
  });
  const data = aligned(keys, all);
  const p = new uPlot(
    {
      width: Math.max(260, target.clientWidth),
      height: 260,
      padding: [16, 16, 0, 0],
      legend: { show: false },
      scales: { x: { time: false } },
      cursor: {
        sync: { key: "adr-flight", setSeries: false },
        drag: { x: true, y: false },
      },
      axes: [
        {
          stroke: "#8996ab",
          grid: { stroke: "#252d3c" },
          ticks: { stroke: "#252d3c" },
          values: (_, ticks) => ticks.map((t) => t.toFixed(1) + "s"),
        },
        {
          stroke: "#8996ab",
          grid: { stroke: "#252d3c" },
          ticks: { stroke: "#252d3c" },
          size: 65,
          values: (_, ticks) => ticks.map((t) => Number(t.toPrecision(4))),
        },
      ],
      series: [
        {},
        ...keys.map((k, i) => ({
          label: pretty(k),
          stroke: palette[i % palette.length],
          width: 1.5,
          points: { show: false },
          spanGaps: false,
        })),
      ],
      hooks: {
        setCursor: [
          (u) => {
            keys.forEach((k, i) => {
              const v =
                u.cursor.idx == null ? null : u.data[i + 1][u.cursor.idx];
              values[i].textContent =
                v == null ? "—" : Number(v.toPrecision(4)).toString();
            });
          },
        ],
      },
    },
    data,
    target,
  );
  inputs.forEach(
    (input, i) =>
      (input.onchange = () => p.setSeries(i + 1, { show: input.checked })),
  );
  const observer = new ResizeObserver(() => {
    if (target.clientWidth > 0)
      p.setSize({ width: Math.max(260, target.clientWidth), height: 260 });
  });
  observer.observe(target);
  observers.push(observer);
  plots.push(p);
}
refresh();
