// src/components/VisionDataLoader.jsx
// "팀원(비전)이 데이터를 준비하면 그것만 넣어서 바로 실행하고 싶다"는 요청 -
// box_scan.json(HMI 설계 가이드라인 4.2절) 또는 all_boxes_corners_*.json
// (준형이 실제로 넘겨준 샘플) 파일을 그대로 선택하면, 백엔드의 vision_adapter가
// 계산에 바로 쓸 수 있는 단순 박스 목록 + 진짜 snapshot_id로 바꿔서 기존
// "박스 목록(JSON)" 파이프라인에 그대로 흘려보낸다.
import { useState } from "react";
import { usePlannerDispatch } from "../state/PlannerContext.jsx";
import { postParseBoxScan, postParseVisionCorners } from "../api/client.js";
import styles from "./VisionDataLoader.module.css";

// 두 스키마를 파일 내용만 보고 구분한다 - 사용자가 "이게 어떤 형식인지" 미리
// 몰라도 팀원이 준 파일을 그대로 올리면 되게 하기 위함. all_boxes_corners_*
// .json에만 있는 corners_m 필드로 먼저 구분하고, 그게 아니면 box_scan.json
// 스키마(frame_id/snapshot_id)로 취급한다.
// File.prototype.text()는 jsdom(테스트 환경)에 아직 구현이 없어서(실제
// 브라우저에서는 됨) FileReader로 읽는다 - 둘 다에서 안정적으로 동작.
function readFileAsText(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(reader.result);
    reader.onerror = () => reject(reader.error);
    reader.readAsText(file);
  });
}

function detectFormat(data) {
  if (Array.isArray(data?.boxes) && data.boxes.length > 0 && "corners_m" in data.boxes[0]) {
    return "vision_corners";
  }
  if (data && ("frame_id" in data || "snapshot_id" in data)) {
    return "box_scan";
  }
  return null;
}

export default function VisionDataLoader({ disabled = false }) {
  const dispatch = usePlannerDispatch();
  const [status, setStatus] = useState("idle"); // idle | loading
  const [fileName, setFileName] = useState("");

  const handleFile = async (e) => {
    const file = e.target.files?.[0];
    e.target.value = ""; // 같은 파일을 연달아 다시 골라도 change 이벤트가 뜨도록 초기화
    if (!file || disabled) return;

    setStatus("loading");
    setFileName(file.name);
    try {
      const text = await readFileAsText(file);
      let data;
      try {
        data = JSON.parse(text);
      } catch {
        throw {
          error_code: "VISION_FILE_JSON_INVALID",
          cause: `${file.name} 파일이 올바른 JSON이 아닙니다.`,
          action: "파일이 손상되지 않았는지 확인한 뒤 다시 시도하세요.",
        };
      }

      const format = detectFormat(data);
      if (!format) {
        throw {
          error_code: "VISION_FORMAT_UNKNOWN",
          cause: "box_scan.json 또는 all_boxes_corners_*.json 형식이 아닙니다.",
          action: "팀원(준형)이 준 비전 결과 파일이 맞는지 확인하세요.",
        };
      }

      const parse = format === "vision_corners" ? postParseVisionCorners : postParseBoxScan;
      const resp = await parse(data);
      dispatch({
        type: "LOAD_VISION_BOXES",
        payload: { boxes: resp.boxes, snapshotId: resp.snapshot_id, sourceLabel: `vision:${resp.snapshot_id}` },
      });
    } catch (err) {
      dispatch({
        type: "COMPUTE_ERROR",
        payload: {
          error_code: err.error_code || "VISION_FILE_INVALID",
          cause: err.cause || err.message || "파일을 처리하지 못했습니다.",
          action: err.action || "파일 내용을 확인한 뒤 다시 시도하세요.",
        },
      });
    } finally {
      setStatus("idle");
    }
  };

  return (
    <div className={styles.loader}>
      <label className={styles.fileButton}>
        {status === "loading" ? "불러오는 중..." : "비전 데이터 파일 불러오기"}
        <input
          type="file"
          accept=".json,application/json"
          data-testid="vision-file-input"
          onChange={handleFile}
          disabled={disabled || status === "loading"}
        />
      </label>
      {fileName && status === "idle" && <span className={styles.fileName}>{fileName}</span>}
      <p className={styles.hint}>
        box_scan.json 또는 all_boxes_corners_*.json 파일을 그대로 선택하면 자동으로 형식을 인식해서 불러옵니다.
      </p>
    </div>
  );
}
