"""MSI1: multiview_scan.py(EDU 저장소, algorism 브랜치)의 순수 검출 로직 이식.

여러 시점에서 모아 m0609_base_link 좌표계로 이미 합쳐진 point cloud(.npy 또는 .ply,
Nx3) 하나에서 box_geometry.py의 프레임-무관 검출 로직으로 박스들의 8개 꼭짓점을
뽑아 dict 리스트로 반환한다. rclpy/cv_bridge 의존 없음(open3d/opencv/numpy만 사용) -
box_scan_action_server.py가 이 결과를 Box3D[] 메시지로 변환한다.

[원본과의 차이 - 의도적으로 제외한 부분]
원본 perception/multiview_scan.py의 save_boxes()(JSON+PLY 파일 저장)와
main()/argparse(CLI 진입점)는 옮기지 않았다 - 알고리즘이 아니라 오프라인 스탠드얼론
실행/디버그 편의 기능이라서다(trunk_map_builder.py가 matplotlib 미리보기 저장을
core/에 포함하지 않은 것과 같은 원칙). Box3D[] 메시지 조립은
box_scan_action_server.py의 책임이다.

좌표계: 입력 point cloud는 이미 m0609_base_link 좌표계라고 가정한다(EDU 저장소
35.crate_scan_setup.py/99.cart_scan_dual_side_holonomic.py가 카메라 world_frame
point cloud를 base_link로 변환해서 scan_cache/merged_cart_scan.npy로 저장함) -
그래서 box_top_extractor.py(단일 시점, 카메라 좌표계 전용)가 하던 camera->base_link
변환이 필요 없다.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import open3d as o3d

from cart2trunk_perception.core import box_geometry as bg

UP_VECTOR = np.array([0.0, 0.0, 1.0], dtype=np.float64)
DOWN_VECTOR = np.array([0.0, 0.0, -1.0], dtype=np.float64)

# box_scan_action_server.py가 Box3D[].header.frame_id로 그대로 쓴다.
OUTPUT_FRAME = "m0609_base_link"

DEBUG_SUPPORT = os.environ.get("CART2TRUNK_DEBUG_SUPPORT", "0") == "1"

# Open3D의 segment_plane()은 시드 고정 없는 RANSAC이라, 완전히 같은(정적인) 전처리된
# point cloud를 다시 검출해도 매번 평면이 다르게 쪼개진다 - 실측 확인: Small처럼
# 노출 면적이 작은 박스는 이 노이즈 때문에 한 번의 검출 시도만으로는 fill_ratio는
# 넘기지만 크기가 완전히 틀어진("그럴듯한 오검출") 조각이 뽑힐 수 있다(같은 입력을
# 5번 반복 검출했더니 footprint가 0.046~0.162 사이를 오갔음, 그 중 진짜 크기(0.13x0.10)에
# 가까운 건 fill_ratio가 가장 높았던 시도뿐이었다). 로봇을 다시 움직이지 않고도
# 같은 병합 point cloud에 대해 검출만 여러 번 반복해서, 같은 물리적 위치에서 나온
# 후보들 중 fill_ratio가 가장 높은(=가장 완전한 사각형에 가까운) 것을 채택한다 -
# run_scan_once.py가 "여러 프레임 관찰 후 최빈값 채택"으로 카메라 프레임 노이즈를
# 우회했던 것과 같은 원리를, 이제는 노이즈의 실제 근원(RANSAC 시드)에 대해 직접 반복한다.
DETECTION_TRIALS = int(os.environ.get("CART2TRUNK_DETECTION_TRIALS", "12"))
# 같은 물리적 박스로 볼 중심 간 거리(m) - 테이블 위 실제 박스 간 최소 간격보다
# 훨씬 작게 잡아서 서로 다른 박스를 하나로 합치지 않게 한다.
DETECTION_GROUP_RADIUS_M = float(os.environ.get("CART2TRUNK_DETECTION_GROUP_RADIUS_M", "0.06"))
# 전체 시도 중 이 비율 미만으로만 나타난 후보는 노이즈(우연히 한 번 걸린 조각)로 보고
# 버린다 - "여러 시도 중 얼마나 일관되게 같은 자리에서 나오는가"도 신뢰도 신호로 쓴다.
DETECTION_MIN_APPEARANCE_FRACTION = float(
    os.environ.get("CART2TRUNK_DETECTION_MIN_APPEARANCE_FRACTION", "0.25")
)

# box_geometry.MIN_BOX_SIDE_M(0.04m)은 일반 파이프라인의 낮은 하한이라(다른
# 용도로 얇은 후보도 통과시켜야 하는 legacy 단일 시점 경로에 영향을 주지 않기 위해
# box_geometry.py 쪽 값은 건드리지 않는다), 실측 확인: 폭 4.5cm짜리 가늘고 긴
# RANSAC 조각(진짜 박스가 아님 - 이 데모의 실제 박스 중 가장 좁은 변도 0.12m)이
# 이 기준을 겨우 통과해서 최종 결과에 유령 4번째 박스로 남는 사례가 있었다.
# multiview_scan 쪽(최종 선택 단계)에서만 더 엄격한 하한을 추가로 적용한다 -
# 실제 박스 카탈로그 중 가장 좁은 변(Medium 0.12~0.13m)보다는 넉넉히 낮게,
# 관측된 유령 조각(0.045m)보다는 확실히 높게 잡는다.
MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M = float(
    os.environ.get("CART2TRUNK_MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M", "0.08")
)

# 88.cart_scan_holonomic.py(카트 바스켓 스캔)에서 실측 확인: 카트 자체의 철망
# 테두리(림)가 하나의 거대한 평면 후보(0.57x0.77m)로 검출되고, 마침 fill_ratio도
# 높게(0.98+) 나와서 유령 박스로 남았다 - 이 데모의 실제 박스는 테이블/카트
# 시나리오를 통틀어 가장 큰 것도 한 변 0.23m를 넘지 않는다. 실제 박스보다는
# 넉넉히 크게, 카트 림처럼 명백히 큰 구조물보다는 확실히 작게 잡는다.
MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M = float(
    os.environ.get("CART2TRUNK_MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M", "0.35")
)

# 같은 이유(3-10절)로 실측 확인: fill_ratio가 매우 낮은(~0.50, 사각형의 절반
# 정도만 실제로 채워진) 후보가 간헐적으로(5회 중 2회) 최소 등장 횟수 기준은
# 통과해서 유령 4번째 박스로 남는 사례가 있었다 - 진짜 박스 윗면은 8회 이상
# 반복 관측에서 항상 fill_ratio 0.9 이상이었다. 최종 선택 단계에서 이 하한
# 미만인 후보도 제외한다.
MIN_FINAL_FILL_RATIO = float(os.environ.get("CART2TRUNK_MIN_FINAL_FILL_RATIO", "0.75"))


# 최종 선택된 박스들 중, 사각형(윗면 footprint)이 겹치는 후보를 한 번 더 정리하기
# 위한 기준 - _group_by_location()의 반경 밖에서도 실제 사각형은 겹칠 수 있어서
# 별도로 둔다. 높이가 이 이내로 같아야만(=진짜 적층이 아니어야만) 적용한다.
DEDUP_OVERLAP_Z_TOLERANCE_M = float(os.environ.get("CART2TRUNK_DEDUP_OVERLAP_Z_TOLERANCE_M", "0.05"))
DEDUP_OVERLAP_RATIO_MIN = float(os.environ.get("CART2TRUNK_DEDUP_OVERLAP_RATIO_MIN", "0.3"))

# 테이블 표면 전체가 우연히 "유효한 평면 후보"로 검출되면(넓고 평평하니 사각형
# 채움비 검사를 쉽게 통과한다), select_support_candidate가 그걸 정상적인 박스
# 지지면으로 착각할 수 있다(SAME_SIZE_STACK_DETECTION_LOG.md 3-6절 실측) - top
# 자신의 면적보다 이 배수 이상 큰 후보는 지지면에서 제외한다. 실제 박스 카탈로그
# 크기 비율(Large/Small 면적비 ~2.8배)보다는 넉넉하게, 테이블 전체 면적비
# (~13~20배)보다는 훨씬 작게 잡는다.
MAX_SUPPORT_AREA_RATIO = float(os.environ.get("CART2TRUNK_MAX_SUPPORT_AREA_RATIO", "6.0"))

# 같은 그룹(같은 물리적 박스) 안에서 시도별로 지지면 탐색이 갈릴 때, "바닥까지
# 뚫고 내려간" 오검출을 걸러내는 상한 - 이 데모의 가장 큰 박스(Large) 높이(0.12m)
# 보다는 넉넉하게, 실측된 오검출 높이(카트 바닥까지, ~0.17~0.21m)보다는 확실히
# 작게 잡는다. detect_boxes_in_base_frame()의 그룹별 최선 선택에서 사용.
STACKED_HEIGHT_PLAUSIBILITY_CEILING_M = float(
    os.environ.get("CART2TRUNK_STACKED_HEIGHT_PLAUSIBILITY_CEILING_M", "0.16")
)

# 88.cart_scan_holonomic.py 적층 시나리오(Large 위에 Medium+Small)에서 실측 확인:
# MAX_SUPPORT_AREA_RATIO를 넉넉히 늘려도, 카트 바닥 자체가 (좁게 크롭된 뒤에도) 위를
# 향한 평면으로 잡히면서 우연히 top 면적 대비 배수 조건을 통과하는 경우가 있었다 -
# 이때 select_support_candidate가 순위를 "median_distance 오름차순"으로 매기므로
# 정상이라면 더 가까운 Large가 이겨야 하는데, SUPPORT_SIZE_PREFERENCE_RATIO(관측
# 점 수 기준 tier)에서 바닥 조각이 Large와 같은 tier로 묶이면 그 안에서 우연히
# 순위가 뒤집힐 수 있다(실측: Medium 높이가 0.11m가 아니라 바닥까지 뚫린 0.21m로
# 복원됨). 실제 적층에서 지지면은 top 바로 아래(대략 top 박스 높이 이내)에 있어야
# 한다는 물리적 사실로 바닥 후보를 아예 배제한다 - 이 데모의 가장 큰 박스(Large)
# 높이(0.12m)보다는 넉넉하게, 카트 바닥까지의 실제 거리(~0.21m)보다는 확실히 작게.
MAX_SUPPORT_RAY_DISTANCE_M = float(os.environ.get("CART2TRUNK_MAX_SUPPORT_RAY_DISTANCE_M", str(bg.MAX_RAY_DISTANCE_M)))
# 실측 확인(88.py 5개 적층 시나리오): allow_plane_only_fallback이 거리만으로 순위를
# 매겨서, 특정 trial에서 진짜 부모가 안 잡히면 카트 반대편의 무관한 박스로 스킵
# 매칭되는 사례가 있었다(box_geometry.select_support_candidate 문서 참고) -
# candidate 자신의 관측 범위 밖 교점은 후보에서 제외해 이를 막는다.
PLANE_ONLY_BOUNDS_MARGIN_M = float(os.environ.get("CART2TRUNK_PLANE_ONLY_BOUNDS_MARGIN_M", "0.05"))

# 88.cart_scan_holonomic.py 5개 적층(2단 피라미드) 시나리오에서 실측 확인: 박스가
# Base+M1+M2+XS1+XS2(5개)+바닥으로 늘어나면서, RANSAC이 한 번의 검출 시도 안에서
# 순회하는 평면 개수 상한(box_geometry.MAX_PLANES=12)을 다 쓰기 전에 작은 박스들
# (M1/M2/XS1/XS2)까지 도달하지 못하는 경우가 잦아졌다(실측: 24회 시도 중 1회만
# 관측). 벽/바닥 조각들(up_alignment 낮아 거절되지만 그 자체로 반복 횟수를 소모)이
# 앞쪽 인덱스를 차지하는 구조라, 박스 개수가 늘수록 더 여유 있는 상한이 필요하다.
MAX_PLANES = int(os.environ.get("CART2TRUNK_MAX_PLANES", str(bg.MAX_PLANES)))
# 실측 확인(88.py 5개 적층 시나리오, margin_growth=0.015): 기본 DBSCAN_EPS_M(2.5cm)이
# 희박한 "다리" 포인트 몇 개만으로도 M1의 실제 표면과 전혀 무관한, 우연히 같은 높이인
# 다른 평평한 영역을 하나의 클러스터로 이어붙였다(60회 시도 중 86%가 실제 크기(0.19m)
# 보다 눈에 띄게 큰 풋프린트로 나옴 - 지지면 매칭에서 XS1/XS2가 잘못된 부모로 스킵
# 매칭되는 근본 원인 중 하나). 1.5cm로 좁히면 과대추정 비율이 86%->29%로 줄고
# 평균 풋프린트도 실측값에 훨씬 가까워진다(더 좁히면(<=1cm) 이번엔 진짜 표면까지
# 조각나서 검출 자체가 실패하기 시작함 - 1.5cm가 실측으로 확인한 최적 지점).
DBSCAN_EPS_M = float(os.environ.get("CART2TRUNK_DBSCAN_EPS_M", "0.015"))

# 5개 적층 시나리오 실측 확인: M1/M2(중간 높이, 옆에서 strafe로 보는 각도)의 RANSAC
# 피팅 평면은 up_alignment가 0.03~0.83으로 기본 임계값(0.94)에 크게 못 미친다 -
# 옆에서 보는 시점 특성상 윗면 점과 옆면 점이 섞여서 법선이 순수 수직에서 벗어난다.
# Small(3개 시나리오의 가장 작은/가장 높은 박스)에서 이미 관측된 문제와 같은
# 계열인데, 박스 개수가 늘고 중간 높이 박스까지 생기면서 더 자주 나타난다.
# 낮추면 벽 등 진짜 옆면까지 섞여 들어올 위험이 있으므로 기본값은 그대로 두고,
# 이 시나리오에서만 환경변수로 완화해서 쓴다.
UP_FACING_NORMAL_DOT_MIN = float(
    os.environ.get("CART2TRUNK_UP_FACING_NORMAL_DOT_MIN", str(bg.UP_FACING_NORMAL_DOT_MIN))
)

# 완화 패스(strict가 못 찾은 나머지 점에서 UP_FACING_NORMAL_DOT_MIN 기준으로
# 추가 검출) 전용 max_planes - strict보다 훨씬 크게 잡아서, 한 번의 RANSAC 호출
# 안에서 중간 크기 박스(strict가 놓친 M1/M2)부터 가장 작은 박스(XS1/XS2)까지
# 순서대로 다 걷어낼 여유를 준다(자세한 이유는 _detect_boxes_once 참고).
RELAXED_MAX_PLANES = int(os.environ.get("CART2TRUNK_RELAXED_MAX_PLANES", str(MAX_PLANES * 3)))


def _debug_log(message: str) -> None:
    print(message, flush=True)


def load_merged_cloud(path: Path) -> np.ndarray:
    path = Path(path)
    if path.suffix == ".npy":
        points = np.load(path)
    elif path.suffix == ".ply":
        pcd = o3d.io.read_point_cloud(str(path))
        points = np.asarray(pcd.points)
    else:
        raise ValueError(f"지원하지 않는 입력 형식: {path.suffix} (.npy 또는 .ply만 가능)")

    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"point cloud shape이 (N,3)이 아닙니다: {points.shape}")
    return points


def _remove_used_points(scene_pcd, used_points_list, tol_m=1e-4):
    """scene_pcd에서 이미 다른 후보에 쓰인 점들을 뺀 나머지만 담은 새 PointCloud를
    반환한다. 실측 확인(중요 버그): candidate.points는 float32로 저장되는데
    scene_pcd는 float64라, 값이 실제로는 같아도(거리 0) 튜플 해시 집합으로 빼면
    dtype이 달라 해시가 안 맞아서 대부분(실측: 6177개 중 556개만) 못 뺐다 -
    KDTree 거리 기반(허용오차 이내는 같은 점으로 취급)으로 바꿔서 dtype과 무관하게
    정확히 뺀다."""
    used_pts = np.vstack([np.asarray(pts, dtype=np.float64) for pts in used_points_list])
    used_pcd = o3d.geometry.PointCloud()
    used_pcd.points = o3d.utility.Vector3dVector(used_pts)
    used_tree = o3d.geometry.KDTreeFlann(used_pcd)

    scene_pts = np.asarray(scene_pcd.points)
    keep_mask = np.ones(len(scene_pts), dtype=bool)
    for i, p in enumerate(scene_pts):
        k, _idx, _dist2 = used_tree.search_radius_vector_3d(p, tol_m)
        if k > 0:
            keep_mask[i] = False

    remaining = o3d.geometry.PointCloud()
    remaining.points = o3d.utility.Vector3dVector(scene_pts[keep_mask])
    return remaining


RELAXED_TOP_Z_PERCENTILE = float(os.environ.get("CART2TRUNK_RELAXED_TOP_Z_PERCENTILE", "95"))
# 실측 확인(88.cart_scan_holonomic.py 5개 적층 시나리오, M1 위에 XS1이 얹힌 경우):
# UP_FACING_NORMAL_DOT_MIN을 0.78까지 완화하면 최대 ~38.7도 기울어진 평면도 "위를
# 향한다"고 통과된다 - 진짜 박스 윗면(수평)이 아니라, M1의 노출 스트립과 그 위
# XS1 가장자리를 동시에 스치는 기울어진(bridging) 평면이 이 관용도 안에서 RANSAC
# 거리 임계값(1cm)을 만족해버릴 수 있다(실측: 풋프린트 대각선 ~0.21m에 걸쳐 높이가
# 최대 11cm까지 변하는 후보가 반복적으로 나왔음). 단순 percentile 보정은 이걸
# "옆면 오염"과 구분 못 해서 XS1의 높이로 통째로 잘못 수렴시켰다(실측: 40회 시도 중
# 진짜 M1 높이(~0.11m)로 잡힌 건 단 1회뿐). 높이 히스토그램의 밀도 골짜기로 구분해
# 보려 했지만, 두 표면이 옆면 점들로 완만하게 이어져 있어(뚜렷한 이봉 분포가
# 아니라 연속된 경사면) 골짜기 자체가 안 잡혔다.
# 대신 점들의 3D 공간적 연결성(DBSCAN)으로 구분한다: 진짜 같은 표면의 점들은
# 서로 가깝게 붙어 있지만(반경 RELAXED_SPLIT_DBSCAN_EPS_M 이내), M1의 노출
# 스트립과 XS1의 오버행 가장자리는 그 사이에 점이 거의 없는 수직 옆면(진짜 gap)으로
# 분리돼 있어 서로 다른 클러스터가 된다 - 높이만 보는 방식과 달리 XY 연결성까지
# 같이 보므로 "완만한 경사면처럼 보이지만 실제로는 끊긴" 경우도 잡아낸다. 여러
# 클러스터로 갈리면 점이 가장 많은(=이 시야에서 가장 안정적으로 관측된, 보통은
# 의도한 자기 자신의 표면) 클러스터만 남기고 나머지는 오염으로 보고 버린 뒤,
# box_geometry.make_candidate()로 그 부분점들만으로 사각형을 다시 피팅한다
# (풋프린트도 오염된 채로 부풀려져 있었으므로 같이 교정됨).
RELAXED_SPLIT_DBSCAN_EPS_M = float(os.environ.get("CART2TRUNK_RELAXED_SPLIT_DBSCAN_EPS_M", "0.02"))
RELAXED_SPLIT_MIN_POINTS = int(os.environ.get("CART2TRUNK_RELAXED_SPLIT_MIN_POINTS", "5"))
# 실측 확인(위 DBSCAN 분리로도 못 잡던 진짜 원인): 이 기울어진 브리징 평면은 M1의
# 노출 스트립과 XS1 오버행 가장자리 사이에 진짜 3D gap이 없다 - 깊이 카메라가
# 전경/배경 경계(실루엣 엣지)에서 흔히 만드는 "flying pixel"(두 깊이값 사이를
# 보간한 유령 점) 노이즈가 그 사이를 연속적으로 채워서, 공간적으로도 끊기지 않은
# 완만한 경사면처럼 보인다 - DBSCAN이 못 가르는 이유. RANSAC 평면 거리 임계값
# (기본 1cm)이 이 브리징을 허용할 만큼 느슨했다 - 실측: 임계값을 6mm로 좁히자
# M1이 20회 시도 중 19회 자기 자신의 진짜 높이(-0.007m)로 정확히 분리되어 나왔다
# (1cm일 때는 20회 중 7회만, 나머지는 XS1 높이로 오검출). strict pass(엄격한
# up_facing_dot)는 원래도 문제없었으므로 건드리지 않고, 완화 패스에만 별도로
# 더 좁은 임계값을 쓴다.
RELAXED_PLANE_DISTANCE_THRESHOLD_M = float(
    os.environ.get("CART2TRUNK_RELAXED_PLANE_DISTANCE_THRESHOLD_M", "0.006")
)


def _split_to_dominant_surface(candidate):
    """DBSCAN으로 공간적으로 끊긴 클러스터를 분리하고, 가장 큰 클러스터만으로
    사각형을 다시 피팅한 새 candidate를 반환한다. 클러스터가 하나뿐이면(오염이
    없거나 완만한 꼬리뿐) 원본을 그대로 반환한다. 재피팅이 실패하면(점이 너무
    적어지는 등) 원본을 그대로 반환한다 - 통째로 버리는 것보다 안전한 쪽을 택함."""
    points = np.asarray(candidate.points, dtype=np.float64)
    if len(points) < RELAXED_SPLIT_MIN_POINTS * 2:
        return candidate

    cluster_pcd = o3d.geometry.PointCloud()
    cluster_pcd.points = o3d.utility.Vector3dVector(points)
    labels = np.asarray(
        cluster_pcd.cluster_dbscan(
            eps=RELAXED_SPLIT_DBSCAN_EPS_M, min_points=RELAXED_SPLIT_MIN_POINTS
        )
    )
    valid_labels = labels[labels >= 0]
    if len(valid_labels) == 0:
        return candidate
    unique, counts = np.unique(valid_labels, return_counts=True)
    if len(unique) <= 1:
        return candidate  # 끊긴 클러스터가 없음 - 오염이 아니라 하나의 표면

    dominant_label = unique[int(np.argmax(counts))]
    dominant_points = points[labels == dominant_label]

    dominant_pcd = o3d.geometry.PointCloud()
    dominant_pcd.points = o3d.utility.Vector3dVector(dominant_points)
    refit = bg.make_candidate(
        candidate.candidate_id,
        dominant_pcd,
        candidate.normal,
        debug=False,
    )
    if refit is None:
        return candidate
    return refit


def _refine_relaxed_top_z(candidate, up_vector, percentile: float = RELAXED_TOP_Z_PERCENTILE) -> None:
    """DBSCAN 분리(_split_to_dominant_surface) 이후에도 남아 있는, 진짜 옆면
    오염(하나로 이어진 표면인데 가장자리 점들이 옆면까지 살짝 걸친 경우 - 실측:
    Z가 최대 7cm까지 퍼짐)에 대한 잔여 보정. 옆면 오염은 점을 항상 실제 윗면보다
    "아래"로만 끌어내리므로, 관측된 점들 중 상위 percentile 값이 RANSAC 평면
    자체보다 더 믿을 만한 윗면 추정치다."""
    pts = np.asarray(candidate.points, dtype=np.float64)
    if len(pts) == 0:
        return
    heights = pts @ up_vector
    corrected_top = float(np.percentile(heights, percentile))
    current_top = float(np.dot(candidate.center, up_vector))
    shift = corrected_top - current_top
    if shift <= 0.0:
        return
    candidate.center = candidate.center + shift * up_vector
    candidate.corners_3d = candidate.corners_3d + shift * up_vector
    normal_up_component = float(np.dot(candidate.normal, up_vector))
    candidate.plane_d = candidate.plane_d - shift * normal_up_component


def _detect_boxes_once(scene_pcd, debug: bool = False) -> list[dict]:
    """box_top_extractor.py의 process_scene_cloud()와 같은 흐름을, 이미 전처리된
    point cloud 하나에 대해 검출 1회(RANSAC 시드 1개) 실행한다.

    2단계(strict -> relaxed) 검출: 실측 확인(88.cart_scan_holonomic.py 5개 적층
    시나리오) - UP_FACING_NORMAL_DOT_MIN을 전역으로 완화해서 한 번에 다 찾으려
    하면(1단계 방식), 옆에서 보는 각도 때문에 옆면이 섞이기 쉬운 작고 높은 박스
    (XS1/XS2)는 가끔 잡히지만, 이미 잘 잡히던 크고 안정적인 박스(Base/M1/M2)의
    평면 피팅까지 같이 노이즈에 오염돼 시도마다 크기가 들쭉날쭉해지고, 지지면
    매칭도 불안정해져 "공중에 뜬 박스"·"다른 박스와 겹치는 박스" 같은 기하학적으로
    말이 안 되는 조합이 나왔다(물리 배치 자체는 서브밀리미터로 정확함을 직접
    측정해 확인 - 검출 알고리즘만의 문제).
    그래서 항상 엄격한 기준(box_geometry 기본값)으로 먼저 찾고, 거기서 찾은 점들을
    빼고 남은 점에서만(원래 신뢰도 높게 잡히던 큰 박스들은 이미 다 골라져서
    없으므로 서로 간섭할 여지가 없다) UP_FACING_NORMAL_DOT_MIN이 완화값일 때만
    2단계로 추가 검출을 한 번 더 돌린다. 완화하지 않았으면(기존 3개 시나리오
    등) 2단계는 그냥 안 돈다 - 동작 완전히 그대로 유지."""
    strict_candidates = bg.detect_box_top_candidates_fixed_up(
        scene_pcd,
        up_vector=UP_VECTOR,
        max_planes=MAX_PLANES,
        dbscan_eps_m=DBSCAN_EPS_M,
        debug=debug,
        debug_log=_debug_log,
    )

    relaxed_candidates = []
    if UP_FACING_NORMAL_DOT_MIN < bg.UP_FACING_NORMAL_DOT_MIN and strict_candidates:
        # 실측 확인(10개 시점 스캔으로 점이 많아진 뒤): 완화 패스를 "찾은 점 다시
        # 빼고 재호출"하는 여러 라운드로 나누면, RANSAC이 한 라운드 안에서 우연히
        # 어떤 표면의 일부만 잡았을 때 나머지 조각이 다음 라운드에서 별개의(더
        # 작은) 후보로 다시 잡혀 하나의 실제 표면이 여러 개로 쪼개지는 부작용이
        # 있었다(실측 확인). 대신 한 번의 호출 안에서 RANSAC 자체가 반복하는 평면
        # 개수 상한(max_planes)을 훨씬 크게 줘서, 같은 반복 안에서 큰 박스부터
        # 작은 박스까지 순서대로 다 걷어낼 기회를 준다 - 회차를 나누지 않으므로
        # 조각남 문제가 없다.
        remaining_pcd = _remove_used_points(scene_pcd, [c.points for c in strict_candidates])
        if len(remaining_pcd.points) >= bg.MIN_PLANE_POINTS:
            relaxed_candidates = bg.detect_box_top_candidates_fixed_up(
                remaining_pcd,
                up_vector=UP_VECTOR,
                max_planes=RELAXED_MAX_PLANES,
                up_facing_dot_min=UP_FACING_NORMAL_DOT_MIN,
                plane_distance_threshold_m=RELAXED_PLANE_DISTANCE_THRESHOLD_M,
                dbscan_eps_m=DBSCAN_EPS_M,
                debug=debug,
                debug_log=_debug_log,
            )
            relaxed_candidates = [_split_to_dominant_surface(c) for c in relaxed_candidates]
            next_id = max(c.candidate_id for c in strict_candidates) + 1
            for c in relaxed_candidates:
                c.candidate_id = next_id
                next_id += 1
                _refine_relaxed_top_z(c, UP_VECTOR)

    candidates = strict_candidates + relaxed_candidates

    # 카메라와의 거리(단일 시점 개념) 대신 높이(Z, 내림차순) - 맨 위 박스부터 처리.
    ordered_candidates = sorted(
        candidates,
        key=lambda candidate: (-float(candidate.center[2]), candidate.candidate_id),
    )

    boxes = []
    for top_candidate in ordered_candidates:
        # allow_plane_only_fallback=True: 여러 다중 시점 배치를 이미 노이즈 저항성
        # 있게 합친 이 오프라인 경로에서만 켠다 - box_top_extractor.py 등 live
        # 단일 시점 경로(box_geometry.select_support_candidate의 다른 모든 호출부)는
        # 이 인자를 넘기지 않아 기존 엄격한 동작 그대로 유지된다.
        support = bg.select_support_candidate(
            top_candidate,
            candidates,
            DOWN_VECTOR,
            support_min_area_ratio=bg.SUPPORT_MIN_AREA_RATIO,
            max_support_area_ratio=MAX_SUPPORT_AREA_RATIO,
            max_ray_distance_m=MAX_SUPPORT_RAY_DISTANCE_M,
            allow_plane_only_fallback=True,
            plane_only_bounds_margin_m=PLANE_ONLY_BOUNDS_MARGIN_M,
            debug=debug,
            debug_log=_debug_log,
        )
        support_type = "box_top"

        if support is None:
            support = bg.detect_floor_boundary(
                top_candidate,
                scene_pcd,
                DOWN_VECTOR,
                debug=debug,
                debug_log=_debug_log,
            )
            support_type = "floor"

        if support is None:
            continue

        corners = bg.compute_box_corners(top_candidate, support, DOWN_VECTOR)
        if corners is None or np.asarray(corners).shape != (8, 3):
            continue
        corners = np.asarray(corners, dtype=np.float32)

        completed_points = bg.generate_completed_box_surface(corners)
        if len(completed_points) == 0:
            continue

        boxes.append(
            {
                "box_id": len(boxes),
                "top": top_candidate,
                "support": support,
                "support_type": support_type,
                "corners": corners,
                "completed_points": completed_points.astype(np.float32),
            }
        )

    return boxes


def _rect_center_xy(box: dict) -> tuple[float, float]:
    """사각형 4꼭짓점의 평균 - box["top"].center(원시 클러스터 점들의 중심)는
    fill_ratio가 낮은(치우친) 조각일수록 실제 사각형 중심과 크게 어긋날 수 있어서
    그룹핑/중복제거에는 항상 이쪽(피팅된 사각형 기준)을 쓴다."""
    corners = box["top"].corners_3d
    return float(corners[:, 0].mean()), float(corners[:, 1].mean())


def _group_by_location(
    all_trial_boxes: list[list[dict]],
    group_radius_m: float,
    z_tolerance_m: float = DEDUP_OVERLAP_Z_TOLERANCE_M,
) -> list[list[dict]]:
    """여러 시도에서 나온 박스들을 (윗면 사각형 중심의 xy 거리 + z 거리 기준) 같은
    물리적 위치끼리 묶는다.

    XY만 보면 안 되는 이유(실측으로 확인한 버그): Small을 Large 바로 위(같은 XY,
    다른 Z)에 스택시킨 경우, XY만으로 그룹을 묶으면 Small의 모든 시도 인스턴스가
    Large 그룹에 합쳐져 버린다 - 그 그룹 안에서 fill_ratio가 항상 더 높은 Large가
    이겨서(각 그룹에서 fill_ratio 최댓값 하나만 채택) Small은 선택될 기회 자체가
    없어진다(적층이 아닌 배치에서는 XY가 겹칠 일이 없어서 이 버그가 안 드러났다).
    35.crate_scan_setup.py의 table_real_boxes dedup에 있던 것과 같은 종류의
    실수라 같은 방식(Z 조건 AND 추가)으로 고친다."""
    groups: list[dict] = []
    for boxes in all_trial_boxes:
        for box in boxes:
            cx, cy = _rect_center_xy(box)
            cz = float(box["top"].center[2])
            placed = False
            for group in groups:
                if (
                    ((cx - group["cx"]) ** 2 + (cy - group["cy"]) ** 2) ** 0.5 < group_radius_m
                    and abs(cz - group["cz"]) < z_tolerance_m
                ):
                    group["items"].append(box)
                    placed = True
                    break
            if not placed:
                groups.append({"cx": cx, "cy": cy, "cz": cz, "items": [box]})
    return [g["items"] for g in groups]


def _box_height_m(box: dict) -> float:
    zs = box["corners"][:, 2]
    return float(zs.max() - zs.min())


def _footprint_aabb(box: dict) -> tuple[float, float, float, float]:
    corners = box["top"].corners_3d
    return float(corners[:, 0].min()), float(corners[:, 0].max()), float(corners[:, 1].min()), float(corners[:, 1].max())


def _aabb_overlap_ratio(a: tuple, b: tuple) -> float:
    """두 xy AABB가 겹치는 면적을, 더 작은 쪽 면적 대비 비율로 반환한다."""
    ax0, ax1, ay0, ay1 = a
    bx0, bx1, by0, by1 = b
    ix0, ix1 = max(ax0, bx0), min(ax1, bx1)
    iy0, iy1 = max(ay0, by0), min(ay1, by1)
    if ix1 <= ix0 or iy1 <= iy0:
        return 0.0
    inter_area = (ix1 - ix0) * (iy1 - iy0)
    area_a = (ax1 - ax0) * (ay1 - ay0)
    area_b = (bx1 - bx0) * (by1 - by0)
    return inter_area / max(1e-9, min(area_a, area_b))


def _dedup_overlapping_footprints(
    boxes: list[dict],
    z_tolerance_m: float = DEDUP_OVERLAP_Z_TOLERANCE_M,
    overlap_ratio_min: float = DEDUP_OVERLAP_RATIO_MIN,
) -> list[dict]:
    """_group_by_location()의 그룹 반경(group_radius_m)보다 두 후보의 사각형
    중심이 더 멀리 떨어져 있어도, 사각형 자체는 겹칠 수 있다 - 실측 확인: RANSAC이
    같은 테이블/박스 표면을 쪼개서 만든 작은 조각의 사각형이 실제 박스(Large)의
    사각형 안쪽에 들어와 있었는데, 두 중심점 거리(0.088m)가 group_radius_m(0.06m)
    보다 커서 별개의 박스로 살아남은 사례가 있었다. 같은 높이(z_tolerance_m 이내)에
    있으면서 사각형이 상당히(overlap_ratio_min 이상) 겹치는 쌍은 fill_ratio가 낮은
    쪽을 버린다 - 높이가 다르면(진짜 적층) 절대 건드리지 않는다."""
    kept: list[dict] = []
    for box in sorted(boxes, key=lambda b: -float(b["top"].fill_ratio)):
        box_z = float(box["top"].center[2])
        box_aabb = _footprint_aabb(box)
        overlaps_kept = any(
            abs(box_z - float(k["top"].center[2])) < z_tolerance_m
            and _aabb_overlap_ratio(box_aabb, _footprint_aabb(k)) >= overlap_ratio_min
            for k in kept
        )
        if overlaps_kept:
            print(
                f"[multiview_scan] 후보 center_xy={np.round(_rect_center_xy(box), 3).tolist()} "
                f"z={box_z:.3f} fill_ratio={box['top'].fill_ratio:.3f}: 같은 높이의 다른(더 신뢰도 높은) "
                f"후보와 사각형이 겹쳐서 제외", flush=True,
            )
            continue
        kept.append(box)
    return kept


# 5개 적층(2단 피라미드) 시나리오 실측 확인: 같은 물리적 박스(M1)가 시도 그룹에
# 따라 서로 다른 높이(0.107m vs 0.062m)로 두 번 살아남는 경우가 있었다 - 둘 다
# STACKED_HEIGHT_PLAUSIBILITY_CEILING_M보다는 작아서(둘 다 "그럴듯해서") 그
# 안전장치로는 못 걸렀고, XY 중심 거리(0.004m)도 DETECTION_GROUP_RADIUS_M 이내라
# _group_by_location() 단계에서부터 다른 그룹으로 안 묶였어야 정상인데, z_tolerance
# (0.045m 차이)가 그룹 경계를 넘어서 별개 그룹으로 갈라졌다. 두 후보의 풋프린트
# 면적은 둘 다 0.15x0.17 안팎으로 거의 같았다 - "위 박스는 항상 아래 박스보다
# 작다"는 설계 규칙상 진짜 다른 두 박스가 겹쳐 있다면 풋프린트 크기가 달라야
# 하므로, 같은 위치에서 풋프린트 면적까지 거의 같은(area_ratio_max 이내) 쌍은
# 높이가 얼마나 다르든 같은 물리적 박스의 중복 오검출로 보고 fill_ratio 높은
# 쪽만 남긴다.
FOOTPRINT_AREA_RATIO_DUPLICATE_MAX = float(
    os.environ.get("CART2TRUNK_FOOTPRINT_AREA_RATIO_DUPLICATE_MAX", "1.35")
)


def _footprint_area(box: dict) -> float:
    x0, x1, y0, y1 = _footprint_aabb(box)
    return max(0.0, x1 - x0) * max(0.0, y1 - y0)


def _dedup_same_footprint_duplicates(
    boxes: list[dict],
    xy_radius_m: float,
    area_ratio_max: float = FOOTPRINT_AREA_RATIO_DUPLICATE_MAX,
) -> list[dict]:
    kept: list[dict] = []
    for box in sorted(boxes, key=lambda b: -float(b["top"].fill_ratio)):
        cx, cy = _rect_center_xy(box)
        area = _footprint_area(box)
        is_dup = False
        for k in kept:
            kx, ky = _rect_center_xy(k)
            dist = ((cx - kx) ** 2 + (cy - ky) ** 2) ** 0.5
            karea = _footprint_area(k)
            if dist < xy_radius_m and area > 0 and karea > 0:
                ratio = max(area, karea) / min(area, karea)
                if ratio <= area_ratio_max:
                    is_dup = True
                    break
        if is_dup:
            print(
                f"[multiview_scan] 후보 center_xy={np.round((cx, cy), 3).tolist()} "
                f"height={_box_height_m(box):.3f} fill_ratio={box['top'].fill_ratio:.3f}: "
                "같은 위치에 풋프린트 면적까지 거의 같은 다른 후보가 있어 같은 물리적 "
                "박스의 중복 검출로 보고 제외", flush=True,
            )
            continue
        kept.append(box)
    return kept


MIN_TRUSTED_FILL_RATIO_FOR_HIDDEN_SEARCH = 0.85


def _split_hidden_same_size_stacks(
    boxes: list[dict],
    scene_pcd,
    debug: bool = False,
) -> list[dict]:
    """"위 박스가 항상 아래 박스보다 작다"는 전제가 깨지는 경우(위아래 박스 크기가
    같음) 대응. 이 경우 아래 박스의 노출면이 없거나 너무 얇아서(4cm 미만)
    detect_box_top_candidates_fixed_up()이 절대 독립 후보로 못 잡는다.

    [설계가 바뀐 이유 - SAME_SIZE_STACK_DETECTION_LOG.md 3-8절 참고]
    처음엔 find_hidden_stacked_box()의 변별 돌출량 히스토그램 하나로 "숨겨진 박스가
    있는지 + 깊이가 얼마인지"를 동시에 찾으려 했다. 실측 확인 결과 이 방식은 노이즈에
    취약했다: 옆면(수직으로 이어지는 면)과 진짜 표면(수평, 뾰족한 밀도 피크)을
    구분하려고 "피크 다음 bin에서 밀도가 떨어지는지" 조건을 추가했더니, 이번엔 진짜
    적층 케이스에서도 그 조건을 계속 통과 못 해서(옆면과 섞인 노이즈 때문에) 못
    찾는 반대쪽 실패가 나타났다.

    대신 detect_floor_boundary()가 원래 갖고 있던 성질을 활용한다: 그 함수는 RANSAC
    평면 후보들 중 "top 바로 아래에서 가장 가까운" 것을 채택하도록 이미 설계돼
    있어서, 숨겨진 동일 크기 박스가 있으면 그 박스 자신의 노출된 윗면을 우연히
    "가장 가까운 지지면"으로 찾아낸다(실측: 15회 반복 중 15회 모두 0.136~0.151m
    범위, 참값 0.14m와 거의 일치). box_geometry.find_stacked_layers()는 이 지지면을
    "혹시 그 자신도 또 다른 지지면 위에 떠 있는 게 아닌지"(=사실은 바닥이 아니라
    또 다른 박스였는지) detect_floor_boundary()를 재귀적으로 다시 호출해서 확인하고,
    노이즈에 대응하기 위해 이 재귀 탐색을 여러 번 반복한 뒤 "몇 겹으로 내려가는가"를
    다수결로 정한다(자세한 실측 수치는 로그 참고).

    depths(오름차순 누적 깊이 리스트)의 길이가 1이면 적층이 아니다(찾은 지지면이
    곧 최종 바닥) - 다만 이 경우에도 다중 시도 median 깊이가 기존 corners 계산에
    쓰인 단일 시도값보다 안정적이므로 항상 재계산해서 교체한다. 길이가 N(>=2)이면
    N-1개의 숨겨진 박스가 있다는 뜻이다 - 각 숨겨진 박스의 XY 오프셋은
    find_hidden_stacked_box()를 forced_depth_m(이미 알고 있는 깊이)와 함께 호출해서
    (히스토그램 탐색 없이) 가장자리 돌출 증거만으로 추정한다.

    fill_ratio가 낮은(신뢰도 낮은) top 후보에 대해서까지 이 탐색을 벌이면, 애초에
    노이즈/유령일 가능성이 높은 후보 위에 또 다른 유령 후보를 만들어 오탐을 배가시킬
    수 있다(실측 확인: fill_ratio=0.716, 12번 중 6번만 나타난 후보에서 실제로
    발생) - 그래서 이미 충분히 신뢰할 만한(fill_ratio가 높은) top 후보에만 이
    탐색을 적용한다.

    [88.cart_scan_holonomic.py 적층 시나리오에서 실측으로 찾은 치명적 버그] 이
    함수는 원래 "select_support_candidate가 지지면을 못 찾아 floor로 떨어진"
    박스만 대상으로 삼을 셈이었는데, 실제로는 support_type을 전혀 안 보고
    fill_ratio만으로 모든 선택된 박스에 적용되고 있었다 - 그래서 이미
    select_support_candidate(+allow_plane_only_fallback)가 Large를 정확히
    지지면으로 찾아서 올바른 corners(높이 0.09m)를 갖고 있던 Medium도 여기서
    다시 find_stacked_layers()로 "혹시 안 보이는 데 또 있나" 재탐색을 당했다.
    find_stacked_layers()는 detect_floor_boundary()의 재귀 호출인데, 이건
    select_support_candidate와 별개의(더 약한) 탐색이라 같은 가려짐 문제에
    걸려 카트 바닥까지 내려갔고, 그 결과로 corners가 통째로 덮어써져서(실측:
    0.09m -> 0.21m) 앞서 제대로 찾은 결과가 조용히 사라졌다. 이 함수의 진짜
    목적(윗박스가 아랫박스를 완전히 가려서 독립 후보 자체가 안 잡히는 경우)에
    맞게, support_type이 이미 "box_top"(다른 박스를 지지면으로 정상적으로
    찾음)인 박스는 건드리지 않는다 - "floor"로 떨어진(=진짜 지지 박스를 못
    찾은) 박스에만 이 재탐색을 적용한다."""
    processed_points = np.asarray(scene_pcd.points)
    result = list(boxes)
    next_synthetic_id = -1

    for box in list(boxes):
        top_candidate = box["top"]
        if top_candidate.fill_ratio < MIN_TRUSTED_FILL_RATIO_FOR_HIDDEN_SEARCH:
            continue
        if box["support_type"] != "floor":
            continue

        depths = bg.find_stacked_layers(
            top_candidate, scene_pcd, DOWN_VECTOR, debug=debug, debug_log=_debug_log,
        )
        if len(depths) == 0:
            continue  # 지지면 자체를 못 찾음 - 기존(단일 시도) 결과를 그대로 둔다

        # depths[-1](다중 시도 median, 기존 단일 시도 raw_depth보다 안정적)로 이
        # top 자신의 corners를 다시 계산한다 - 적층 여부와 무관하게 정확도가 개선된다.
        terminal_support = bg.flat_plane_support_at_depth(top_candidate, depths[-1], DOWN_VECTOR)
        recomputed_corners = bg.compute_box_corners(top_candidate, terminal_support, DOWN_VECTOR)
        recomputed_corners = np.asarray(recomputed_corners) if recomputed_corners is not None else None
        if recomputed_corners is not None and recomputed_corners.shape == (8, 3):
            recomputed_completed = bg.generate_completed_box_surface(recomputed_corners.astype(np.float32))
            if len(recomputed_completed) > 0:
                box["corners"] = recomputed_corners.astype(np.float32)
                box["completed_points"] = recomputed_completed.astype(np.float32)

        num_hidden = len(depths) - 1
        if num_hidden == 0:
            continue

        print(
            f"[multiview_scan] top={top_candidate.candidate_id}: 재귀 바닥 재탐색으로 숨겨진 "
            f"동일 크기 박스 {num_hidden}개 발견(누적 깊이={[round(d, 4) for d in depths]})",
            flush=True,
        )

        # 각 숨겨진 박스의 top 표면 = depths[i]에서 원래 top의 XY 오프셋을
        # find_hidden_stacked_box(forced_depth_m=depths[i])로 재추정한 것.
        # 마지막 숨겨진 박스의 지지면만 진짜 터미널(depths[-1]); 중간 박스들의
        # 지지면은 그 다음 깊이의 top(forced_depth_m=depths[i+1]).
        layer_tops = []
        for i in range(num_hidden):
            layer_top = bg.find_hidden_stacked_box(
                top_candidate,
                processed_points,
                DOWN_VECTOR,
                forced_depth_m=depths[i],
                debug=debug,
                debug_log=_debug_log,
            )
            if layer_top is None:
                # 가장자리 돌출 증거를 전혀 못 찾은 극단적 경우(현재 구현은
                # require_offset_evidence=False라 이 분기는 사실상 도달하지 않지만,
                # 방어적으로 top의 footprint를 오프셋 0으로 그대로 사용한다.
                layer_top = bg.flat_plane_support_at_depth(top_candidate, depths[i], DOWN_VECTOR)
            layer_top.candidate_id = next_synthetic_id
            next_synthetic_id -= 1
            layer_tops.append(layer_top)

        # 원래 top 자신의 support를 "먼 바닥"에서 "첫 번째로 찾은 숨겨진 박스"로 교체.
        corrected_top_corners = bg.compute_box_corners(top_candidate, layer_tops[0], DOWN_VECTOR)
        corrected_top_corners = np.asarray(corrected_top_corners) if corrected_top_corners is not None else None
        if corrected_top_corners is not None and corrected_top_corners.shape == (8, 3):
            corrected_completed = bg.generate_completed_box_surface(corrected_top_corners.astype(np.float32))
            if len(corrected_completed) > 0:
                old_height = float(np.mean(box["corners"][:4, 2] - box["corners"][4:, 2]))
                new_height = float(np.mean(corrected_top_corners[:4, 2] - corrected_top_corners[4:, 2]))
                print(
                    f"[multiview_scan] top={top_candidate.candidate_id}: 원래 박스 높이 "
                    f"{old_height:.3f}m -> {new_height:.3f}m로 보정(지지면을 숨겨진 박스로 교체)",
                    flush=True,
                )
                box["corners"] = corrected_top_corners.astype(np.float32)
                box["completed_points"] = corrected_completed.astype(np.float32)
                box["support"] = layer_tops[0]
                box["support_type"] = "box_top"

        # 숨겨진 박스들을 각각 독립 박스로 등록 - i번째 박스의 지지면은
        # (i+1)번째 층의 top(마지막이면 진짜 터미널 바닥).
        for i in range(num_hidden):
            this_top = layer_tops[i]
            if i + 1 < num_hidden:
                this_support = layer_tops[i + 1]
                this_support_type = "box_top"
            else:
                this_support = bg.flat_plane_support_at_depth(top_candidate, depths[-1], DOWN_VECTOR)
                this_support_type = "floor"

            hidden_corners = bg.compute_box_corners(this_top, this_support, DOWN_VECTOR)
            hidden_corners = np.asarray(hidden_corners) if hidden_corners is not None else None
            if hidden_corners is None or hidden_corners.shape != (8, 3):
                print(
                    f"[multiview_scan] 숨겨진 박스 후보(top={top_candidate.candidate_id} 아래, "
                    f"층 {i})의 8꼭짓점 복원 실패 -> 무시", flush=True,
                )
                continue
            hidden_completed = bg.generate_completed_box_surface(hidden_corners.astype(np.float32))
            if len(hidden_completed) == 0:
                continue

            result.append(
                {
                    "box_id": -1,  # 호출부에서 전체 재부여
                    "top": this_top,
                    "support": this_support,
                    "support_type": this_support_type,
                    "corners": hidden_corners.astype(np.float32),
                    "completed_points": hidden_completed.astype(np.float32),
                }
            )

    return result


# 실측 확인(사용자 지적: 완성된 PLY에서 Medium-Small 사이에 눈에 보이는 빈 공간이
# 있음): 각 박스 자신의 윗면(top)은 이미 mm 단위로 정확하게 검출되는데, 그 박스를
# 받치는 지지면(support)은 별도의 ray-cast/평면 재탐색으로 얻어지는 값이라 그
# 부모 박스 자신의(마찬가지로 정확한) 윗면과 정확히 일치한다는 보장이 없다(실측:
# M1-XS1 사이 1.0cm, M2-XS2 사이 2.4cm 틈). 이미 최종 선택된 5개 박스는 전부
# 신뢰도 높게 검출됐으므로, 지지면을 다시 추정하지 않고 "이 박스 바로 아래, XY가
# 겹치는 다른 검출된 박스들 중 가장 높은 것"의 윗면 z에 바닥을 직접 스냅해서 이
# 틈을 원천적으로 없앤다 - 윗면(첫 4개 코너)은 그대로 두고 바닥(마지막 4개 코너)
# 만 옮기므로, 이미 정확한 윗면 위치는 전혀 건드리지 않는다.
MIN_PARENT_OVERLAP_RATIO = float(os.environ.get("CART2TRUNK_MIN_PARENT_OVERLAP_RATIO", "0.3"))


def _snap_bottoms_to_detected_parents(
    boxes: list[dict], min_overlap_ratio: float = MIN_PARENT_OVERLAP_RATIO
) -> list[dict]:
    for box in boxes:
        box_top_z = float(box["corners"][:4, 2].mean())
        box_aabb = _footprint_aabb(box)
        best_parent = None
        best_parent_top_z = -np.inf
        for other in boxes:
            if other is box:
                continue
            other_top_z = float(other["corners"][:4, 2].mean())
            if other_top_z >= box_top_z - 0.01:
                continue  # 이 박스보다 위/비슷한 높이면 부모가 될 수 없음
            if _aabb_overlap_ratio(box_aabb, _footprint_aabb(other)) < min_overlap_ratio:
                continue
            if other_top_z > best_parent_top_z:
                best_parent_top_z = other_top_z
                best_parent = other
        if best_parent is None:
            continue

        current_bottom_z = float(box["corners"][4:, 2].mean())
        shift = best_parent_top_z - current_bottom_z
        if abs(shift) < 1e-6:
            continue

        corners = box["corners"].copy()
        corners[4:, 2] += shift
        box["corners"] = corners.astype(np.float32)
        completed = bg.generate_completed_box_surface(box["corners"])
        if len(completed) > 0:
            box["completed_points"] = completed.astype(np.float32)
        print(
            f"[multiview_scan] box_id={box['box_id']}: 바닥을 부모(box_id={best_parent['box_id']}) "
            f"윗면에 스냅 (이동량={shift * 1000:.1f}mm)",
            flush=True,
        )
    return boxes


def detect_boxes_in_base_frame(
    points_base: np.ndarray,
    debug: bool = DEBUG_SUPPORT,
    trials: int = DETECTION_TRIALS,
    group_radius_m: float = DETECTION_GROUP_RADIUS_M,
    min_appearance_fraction: float = DETECTION_MIN_APPEARANCE_FRACTION,
) -> list[dict]:
    """base_link 좌표계로 이미 합쳐진 point cloud 하나에서 박스를 검출한다.

    Open3D RANSAC이 시드 고정이 없어 같은 입력도 검출마다 평면이 다르게 쪼개지는
    문제(특히 작은 박스에서 fill_ratio는 통과하지만 크기가 틀어진 조각이 나올 수
    있음, perception/STACKED_BOX_DETECTION_DEBUG_GUIDE.md 6절과 동일 계열) 때문에,
    전처리는 한 번만 하고 검출 자체를 `trials`번 반복한다. 같은 물리적 위치(xy 근접)에서
    나온 후보들을 묶어서 그중 fill_ratio가 가장 높은(가장 완전한 사각형에 가까운) 것을
    채택하고, 전체 시도 중 너무 적게(< min_appearance_fraction) 나타난 후보는
    우연히 걸린 조각으로 보고 버린다."""
    scene_pcd = bg.preprocess_cloud(points_base)
    processed_points = np.asarray(scene_pcd.points)
    print(
        f"[multiview_scan] 입력 {len(points_base)}점 -> 전처리 후 {len(processed_points)}점",
        flush=True,
    )

    if len(processed_points) < bg.MIN_PLANE_POINTS:
        print("[multiview_scan] 전처리 후 point가 너무 적습니다 - 검출 생략", flush=True)
        return []

    # 실측 확인(88.cart_scan_holonomic.py 적층 시나리오): Open3D의 segment_plane()은
    # "시드 고정 없음"이라고 알려져 있지만, 프로세스 시작 시 초기화된 전역 RNG
    # 상태(o3d.utility.random)를 그대로 이어 쓰기 때문에 한 프로세스 안에서 반복
    # 호출한 24번의 trial이 서로 강하게 상관돼 있었다 - 실제로 한 번의 run_scan_batch.py
    # 실행 안에서는 24번 다 좋은 결과(또는 24번 다 나쁜 결과)가 몰리고, 다른 실행에서는
    # 정반대로 몰리는 극단적인 편차가 관찰됐다(하나의 실행 안에서 다양성이 없으면,
    # trials/min_appearance_fraction으로 "우연히 걸린 조각" 노이즈를 걸러내려는 설계
    # 의도 자체가 무력화된다). 매 trial 시작 전에 OS 엔트로피로 전역 시드를 다시 심어서
    # trial마다 독립적인 RANSAC 샘플링이 되게 한다.
    all_trial_boxes = []
    for _ in range(trials):
        # o3d.utility.random.seed()는 부호 있는 32비트 int만 받는다 - os.urandom(4)를
        # 부호 없는 정수로 그대로 넘기면 절반 확률로 int32 최댓값(2**31-1)을 넘어서
        # "incompatible function arguments" 예외로 죽는다(실측 확인: 그래서 여러 번의
        # "성공한 것처럼 보인" 재현이 사실은 매번 크래시해서 새로 저장을 못 하고
        # 이전 결과 파일을 계속 읽고 있었을 뿐이었다).
        o3d.utility.random.seed(int.from_bytes(os.urandom(4), "little") % (2**31 - 1))
        all_trial_boxes.append(_detect_boxes_once(scene_pcd, debug=debug))
    trial_counts = [len(b) for b in all_trial_boxes]
    print(
        f"[multiview_scan] 검출 {trials}회 반복, 시도별 박스 개수={trial_counts}",
        flush=True,
    )

    groups = _group_by_location(all_trial_boxes, group_radius_m)
    min_appearances = max(1, int(np.ceil(trials * min_appearance_fraction)))

    # 참고: 같은 시도(trial) 안에서도 하나의 물리적 평면이 근접한 조각 여러 개로
    # 쪼개져 같은 그룹에 들어갈 수 있어서, len(items)가 trials를 넘을 수도 있다 -
    # "이 위치에서 총 몇 개의 후보 인스턴스가 나왔나"이지 "몇 번의 시도에서
    # 나타났나"는 아니다. 그래도 min_appearances 기준(전체 시도 수 대비 비율)으로
    # 보는 것 자체는 유효하다 - 우연히 한두 번만 걸린 조각과, 어떤 형태로든
    # 지속적으로 나타나는 진짜 물체를 구분하는 목적이기 때문이다.
    selected = []
    for items in groups:
        if len(items) < min_appearances:
            rep = items[0]["top"]
            print(
                f"[multiview_scan] 후보 center={np.round(rep.center, 3).tolist()}: "
                f"{len(items)}개 인스턴스만 관측(< {min_appearances}, {trials}회 시도 기준) -> 노이즈로 보고 제외",
                flush=True,
            )
            continue
        # 실측 확인(88.cart_scan_holonomic.py 적층 시나리오): fill_ratio는 "윗면
        # 사각형이 얼마나 잘 채워졌는가"만 볼 뿐 "아래쪽 지지면을 제대로 찾았는가"와는
        # 무관하다 - 같은 물리적 박스라도 시도(trial)마다 지지면 탐색 결과가 갈릴 수
        # 있어서(가려진 영역이라 어떤 시도는 진짜 지지 박스를 찾고 어떤 시도는 바닥까지
        # 뚫고 내려간다), fill_ratio만 보고 고르면 우연히 fill_ratio가 더 높았던
        # "잘못 떨어진" 인스턴스가 선택되어 박스 높이가 실제보다 훨씬 크게(예: 0.11m여야
        # 할 게 0.21m) 복원되는 사례가 있었다. 처음엔 support_type=="box_top"을
        # 우선하려 했지만, allow_plane_only_fallback도 "다른 top 후보"에 매칭되면
        # 똑같이 "box_top"으로 표시되므로(그 다른 후보가 진짜 지지 박스가 아니라
        # 우연히 위를 향한 바닥 조각이어도 구분 못 함) 신뢰할 수 없었다 - 대신 "바닥까지
        # 뚫고 내려간 오검출은 항상 높이를 실제보다 부풀리기만 한다(줄이는 방향으로는
        # 절대 안 틀림)"는 물리적 사실을 직접 이용한다: 같은 그룹 안에서 완성된 박스
        # 높이가 STACKED_HEIGHT_PLAUSIBILITY_CEILING_M 이내인 인스턴스를 우선하고,
        # 그 안에서 fill_ratio로 타이브레이크한다(전부 이 상한을 넘으면 어쩔 수 없이
        # 기존처럼 fill_ratio 1위를 그대로 씀).
        plausible = [b for b in items if _box_height_m(b) <= STACKED_HEIGHT_PLAUSIBILITY_CEILING_M]
        pool = plausible if plausible else items
        best = max(pool, key=lambda b: (b["top"].fill_ratio, len(b["top"].points)))
        narrower_side = min(float(best["top"].width), float(best["top"].height))
        wider_side = max(float(best["top"].width), float(best["top"].height))
        if narrower_side < MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M:
            print(
                f"[multiview_scan] 후보 center={np.round(best['top'].center, 3).tolist()}: "
                f"짧은 변이 {narrower_side:.3f}m(<{MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M}m)로 너무 가늘어서 "
                "실제 박스로 보기 어려움 -> 제외", flush=True,
            )
            continue
        if wider_side > MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M:
            print(
                f"[multiview_scan] 후보 center={np.round(best['top'].center, 3).tolist()}: "
                f"긴 변이 {wider_side:.3f}m(>{MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M}m)로 너무 커서 "
                "실제 박스로 보기 어려움(카트/테이블 등 구조물로 추정) -> 제외", flush=True,
            )
            continue
        if float(best["top"].fill_ratio) < MIN_FINAL_FILL_RATIO:
            print(
                f"[multiview_scan] 후보 center={np.round(best['top'].center, 3).tolist()}: "
                f"fill_ratio={best['top'].fill_ratio:.3f}(<{MIN_FINAL_FILL_RATIO})로 사각형이 "
                "부실하게 채워져 실제 박스로 보기 어려움 -> 제외", flush=True,
            )
            continue
        fill_ratios = [round(float(b["top"].fill_ratio), 3) for b in items]
        print(
            f"[multiview_scan] 후보 center={np.round(best['top'].center, 3).tolist()}: "
            f"{len(items)}개 인스턴스 관측, fill_ratio 분포={fill_ratios} -> "
            f"fill_ratio={best['top'].fill_ratio:.3f}인 것 채택",
            flush=True,
        )
        selected.append(best)

    before_overlap_dedup = len(selected)
    selected = _dedup_overlapping_footprints(selected)
    if len(selected) < before_overlap_dedup:
        print(
            f"[multiview_scan] 사각형 겹침 정리: {before_overlap_dedup} -> {len(selected)}개",
            flush=True,
        )

    before_footprint_dedup = len(selected)
    selected = _dedup_same_footprint_duplicates(selected, group_radius_m)
    if len(selected) < before_footprint_dedup:
        print(
            f"[multiview_scan] 같은 풋프린트 크기 중복 정리: {before_footprint_dedup} -> {len(selected)}개",
            flush=True,
        )

    before_split = len(selected)
    selected = _split_hidden_same_size_stacks(selected, scene_pcd, debug=debug)
    if len(selected) > before_split:
        print(
            f"[multiview_scan] 동일 크기 적층 분리: {before_split} -> {len(selected)}개",
            flush=True,
        )

    # 높이(Z, 내림차순) 순으로 box_id 재부여 - 맨 위 박스부터.
    selected.sort(key=lambda b: -float(b["top"].center[2]))
    for i, box in enumerate(selected):
        box["box_id"] = i

    selected = _snap_bottoms_to_detected_parents(selected)

    print(f"[multiview_scan] 복원된 박스 {len(selected)}개", flush=True)
    return selected
