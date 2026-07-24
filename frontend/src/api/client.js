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
