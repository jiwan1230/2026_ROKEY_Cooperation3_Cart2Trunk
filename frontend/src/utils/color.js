// src/utils/color.js
// 백엔드 algorism_bridge.color_for_box_id()와 동일한 알고리즘(CRC32 해시로
// hue를 0~360 전체에서 결정론적으로 뽑고, 채도/명도도 살짝 흔드는 방식) -
// 같은 box_id는 Before(대기 중, 아직 계산 전)든 After(배치 후, 백엔드가
// score와 함께 계산해준 결과)든 항상 같은 색을 유지해야 하므로, 서버 응답을
// 기다리지 않고도 프론트엔드에서 바로 색을 계산할 수 있게 그대로 옮겨왔다.
// (box_id는 이 프로젝트 전체에서 항상 ASCII라서 UTF-8 바이트와 UTF-16 코드
// 유닛이 1:1로 일치 - 그 전제 하에 파이썬의 zlib.crc32(id.encode())와
// 동일한 결과를 낸다.)

const CRC32_TABLE = (() => {
  const table = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) {
      c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    }
    table[n] = c >>> 0;
  }
  return table;
})();

export function crc32(str) {
  let crc = 0xffffffff;
  for (let i = 0; i < str.length; i++) {
    crc = CRC32_TABLE[(crc ^ str.charCodeAt(i)) & 0xff] ^ (crc >>> 8);
  }
  return (crc ^ 0xffffffff) >>> 0;
}

function hslToHex(h, s, l) {
  s /= 100;
  l /= 100;
  const k = (n) => (n + h / 30) % 12;
  const a = s * Math.min(l, 1 - l);
  const f = (n) => l - a * Math.max(-1, Math.min(k(n) - 3, Math.min(9 - k(n), 1)));
  const toHex = (x) => Math.round(x * 255).toString(16).padStart(2, "0");
  return `#${toHex(f(0))}${toHex(f(8))}${toHex(f(4))}`;
}

export function colorForBoxId(boxId) {
  const h = crc32(String(boxId));
  const hue = h % 360;
  const sat = 55 + (Math.floor(h / 360) % 20); // 55~74%
  const light = 45 + (Math.floor(h / 7200) % 20); // 45~64%
  return hslToHex(hue, sat, light);
}
