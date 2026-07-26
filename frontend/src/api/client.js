const BASE = "/api";

async function handleResponse(resp) {
  const body = await resp.json();
  if (!resp.ok) {
    const err = new Error(body.cause || "요청이 실패했습니다");
    err.error_code = body.error_code;
    err.cause = body.cause;
    err.action = body.action;
    throw err;
  }
  return body;
}

export async function fetchTrunkMaps() {
  const resp = await fetch(`${BASE}/trunk-maps`);
  const body = await handleResponse(resp);
  return body.trunk_maps;
}

export async function fetchBoxPresets() {
  const resp = await fetch(`${BASE}/box-presets`);
  const body = await handleResponse(resp);
  return body.presets;
}

export async function postPlan(requestBody) {
  const resp = await fetch(`${BASE}/plan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestBody),
  });
  return handleResponse(resp);
}

export async function postApprove(requestBody) {
  const resp = await fetch(`${BASE}/approve`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestBody),
  });
  return handleResponse(resp);
}

export async function postSend(requestBody) {
  const resp = await fetch(`${BASE}/send`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestBody),
  });
  return handleResponse(resp);
}

// 비전(준형)이 만드는 box_scan.json(HMI 설계 가이드라인 4.2절 스키마)을
// 계획 계산에 바로 쓸 수 있는 단순 박스 목록으로 변환한다.
export async function postParseBoxScan(boxScanJson) {
  const resp = await fetch(`${BASE}/parse-box-scan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(boxScanJson),
  });
  return handleResponse(resp);
}

// 준형이 실제로 넘겨주는 all_boxes_corners_*.json 스키마용.
export async function postParseVisionCorners(visionCornersJson) {
  const resp = await fetch(`${BASE}/parse-vision-corners`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(visionCornersJson),
  });
  return handleResponse(resp);
}
