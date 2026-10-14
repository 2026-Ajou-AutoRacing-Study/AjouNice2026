# 보행자 연결 개선과 LiDAR 이동 비교

상태: EVIDENCE. 검증일: 2026-10-14. 현재 실행 기준은 [CURRENT](CURRENT.md)를 따른다.
일반 보행자 연결 순위 개선은 운영 경로에 적용했다. LiDAR 50개와 회전 경계 묶음은
저속 이동 검증을 보완했지만 기본 51개를 유지한다. E24 가중치·threshold·학습·기존 5K는 변경하지 않았다.

## 보행자 순위 변경의 범위

기존 XY 거리·Mahalanobis 허용 조건을 통과한 pedestrian 연결 후보만 Gaussian XY 순위로
정렬한다. 공분산이 지나치게 넓은 임시 track이 기존 사람의 관측을 가져가는 현상을 줄인다.
허용 범위, 클래스 제한, 생성·확정·삭제, coasting, duplicate birth guard는 유지한다.
낮은 confidence의 측정 공분산 10배도 기존 update와 같은 기준이다. 유효하지 않은
공분산이나 비유한 순위는 원래 비용으로 복귀하며 새로운 연결을 허용하지 않는다.

Tracker submodule revision은 `1cdf0a9`다. `object_pipeline.launch`의
`pedestrian_gaussian_association_ranking`과 `tracking_prediction.launch`의
`tracker_pedestrian_gaussian_association_ranking`은 기본 ON이다. generic C++ core,
standalone tracker launch와 YAML 기본은 OFF여서 기존 직접 실행의 의미를 보존한다.
운영 alias와 validator가 사용하는 object pipeline은 같은 ON 경로를 통과한다.
시작 로그의 `pedestrian_gaussian_rank=1`로 실제 binary 반영을 확인한다.

## 같은 입력의 코어 대조

원래 3,647장, 새 세 장면 1,058장, 최신 팀장님 bag의 detection 1,859프레임을 고정했다.
팀장님 bag의 34프레임은 원본 TF 보간이 없어 코어 대조에서 제외했다.
새 코어 OFF 출력은 수정 전 보존 binary와 정확히 같았다. ON의 차량·장애물 출력도
OFF와 같았으며 보행자 생성 수가 달라지는 숫자 ID counter만 비교에서 제외했다.

| 표본 | 보행자 ID 교체 후보 OFF→ON | 한 정답 근처 다중 ID 프레임 | 사람 대응 수 | XY 오차 평균 |
|---|---:|---:|---:|---:|
| 새 같은 방향 두 사람, 351장 | 4→0 | 54→6 | 698→698 | 0.17466→0.17673m |
| 새 반대 방향 두 사람, 354장 | 1→0 | 0→0 | 704→704 | 0.13651→0.13622m |
| 팀장님 완주 bag, 유효 1,825장 | 2→0 | 16→0 | 114→114 | 0.41426→0.42907m |

같은 방향 새 장면의 p95는 두 조건 모두 0.25078m다. 팀장님 bag의 평균은 약 1.48cm
악화됐지만 중앙값은 0.27602→0.27505m, p95는 두 조건 모두 1.22489m였다.
모든 위치 지표가 개선된 것은 아니며, 정상 사람 대응 유지와 ID 안정성의 절충으로 적용했다.
카탈로그 pose 대응은 visibility-qualified native GT/AP와 구분한다.

코어 결과는 `planner_logs/analysis/261014_pedestrian_rank_promotion_v1/` 아래
`regression_initial/report.json`, `regression_fresh/report.json`에 보존한다.
새 원본은 `261014_pedestrian_rank_followup_live_v1`, 고정 입력은
`261014_pedestrian_rank_followup_inputs_v1`, E24 nearest 전체 추론은
`261014_pedestrian_rank_followup_inference_v1`이다.

## 실제 ROS 노드와 복합체 관측

별도 ROS master·clock에서 설치된 두 tracker에 동일한 world-frame detection을 전달했다.
현재 gravity-center 표현을 사용하고 매 장면 fresh node로 시작했다. 세 장면의 입력은
351/354/353개이고 30Hz 출력은 각각 양쪽 모두 906/907/904개였다.

| 실제 ROS 재생 | ID 교체 후보 OFF→ON | 한 정답 근처 다중 ID 프레임 | 사람 대응 수 | XY 평균 OFF→ON |
|---|---:|---:|---:|---:|
| 같은 방향 | 4→0 | 138→15 | 1,772→1,774 | 0.17626→0.17849m |
| 반대 방향 | 1→0 | 0→0 | 1,776→1,776 | 0.13859→0.13791m |

반려동물 동반 자산은 따로 기록하며 검출 단계에서 임의로 제외하지 않는다. 실제 ROS
904출력 중 작은 높이 박스 존재는 598→570프레임, 작은 박스 중복은 70→17프레임이다.
큰 박스 존재는 899→898, 중복은 145→0이다. 높이 구분은 dog/person 정답이 아니므로
작은 박스 28프레임 감소를 정상 구성 객체 누락 또는 중복 제거로 확정하지 않는다.
일반 사람의 명확한 개선을 이 불확실성만으로 계속 보류하지 않되, 복합체는 후속 관측한다.

실제 ROS 자료는 같은 promotion 폴더의 `ros_pair_follow`, `ros_pair_opposed`, `ros_pet`에 둔다.
이 재생은 실제 message/node/scheduler 검증이며 새 LAN 입력·GPU queue·planner 폐루프는 아니다.

## LiDAR 저속 이동 입력

`run_lidar_short_motion_probe.py`는 기존 MGeo link의 8m 목표 구간을 1.5m/s 이하로
따라가는 독립 QA 제어기다. 고주로·로터리 각각 51/50/cut0을 재로드해 총 여섯 bag을 저장했다.
기존 NPC spawn은 유지했고 접촉, stale Ego/Object, 가까운 객체, 도로 이탈, 과속이면 중단한다.
최대 횡편차는 0.258m 이내였고 마지막 실제 gear=1, 속도 약 0.000007m/s를 확인했다.
기존 전체 미션의 TIMEOUT을 해결하거나 그 미션을 완주한 결과로 해석하지 않는다.

| 구간 | 정책 | header→PointCloud 수신 평균 | p95 |
|---|---|---:|---:|
| 고주로 | 51 | 86.81ms | 100.26ms |
| 고주로 | 50 | 42.41ms | 47.41ms |
| 고주로 | cut0 | 42.19ms | 47.97ms |
| 로터리 | 51 | 80.37ms | 90.12ms |
| 로터리 | 50 | 38.15ms | 43.41ms |
| 로터리 | cut0 | 38.83ms | 44.79ms |

51개는 이번 원본에서 367.2°·서로 다른 footer 2개를 포함했다. 실제 50/cut0은
360°·footer 1개였다. 이 수신 age 차이는 순수 GPU 시간이나 정확한 물리 센서 지연이 아니다.
50/cut0의 첫 패킷 host stamp도 source footer보다 평균 약 37~41ms 이르므로
묶음 변경과 camera/pose 참조 시점 변경을 분리한다.

## 같은 원본으로 E24 비교

고주로 62개·로터리 73개의 공통 source footer를 선택했다. 원래 51개 cloud를 설치된
decoder로 재생한 209장은 모든 필드가 원본과 정확히 같았다. 동일 원본에서
51/50/cut0/cut180/실험 footer 참조를 각각 전체 forward했고, 카메라와 참조 시간을 고정한
별도 기하 비교도 수행했다. footer 참조는 이 PC의 실험 기준이지 LAN 시간 oracle이 아니다.

51개 녹화의 중간 패킷에서 고정 50개씩 재조립하면 시작 위상이 유지돼 이번 대조에서도
footer 두 개가 섞였다. 새 live driver의 위상이 맞은 고정 50개와 동일한 조건으로 취급하지 않는다.
cut0은 같은 원본 재조립에서도 footer 하나였다. 카메라·참조 시간을 고정해도 차량 대응
출력 차이 p95는 고주로 50/cut0 각각 0.016/0.783m, 로터리 0.592/0.701m였다.
이는 정답 오차가 아니라 두 예측 사이 차이다. 같은 cut0 입력을 반복한 135쌍의 raw 출력은
모두 정확히 같았다. 따라서 모든 차이를 camera 선택이나 추론 반복 변동으로 설명하지 않는다.

| 구간·정책 | 차량 pose 대응 | XY 오차 평균 | p95 |
|---|---:|---:|---:|
| 고주로 51 | 119/153 | 0.47790m | 0.92144m |
| 고주로 50 | 118/153 | 0.49082m | 0.93321m |
| 고주로 cut0 | 119/153 | 0.29220m | 0.50764m |
| 로터리 51 | 81/94 | 0.25374m | 0.65498m |
| 로터리 50 | 81/94 | 0.15942m | 0.32613m |
| 로터리 cut0 | 85/95 | 0.30732m | 0.47164m |

분모는 각 header 시점의 ROI·카탈로그 대응이며 AP가 아니다. cut0의 로터리 평균은
51보다 나쁘고, baseline에 없던 Obstacle 출력도 두 건 생겼다. 이를 바로 FP로 단정하지 않는다.
각 방식의 코어 재생은 고주로 차량 ID 3개·로터리 2개, 교체 후보·다중 ID는 모두 0이었다.
저속·짧은 구간에서는 연결을 유지했지만 한 방식의 전 구간 우월성은 확인하지 못했다.
기본은 51개로 유지하고, 고속 Ego·LAN·실제 시간 정합을 포함한 별도 비교 후 승격한다.

원본·packet audit는 `261014_lidar_short_motion_v1`, decoded 입력은
`261014_lidar_short_motion_decoded_v1`, 추론·분석은 `261014_lidar_short_motion_inference_v1`,
`261014_lidar_short_motion_analysis_v1.json`, 코어는 `261014_lidar_short_motion_tracking_v1`이다.
기하 고정 입력과 추론은 `261014_lidar_short_motion_geometry_inputs_v1`,
`261014_lidar_short_motion_geometry_inference_v2`, 기하 분석과 tracking은
`261014_lidar_short_motion_geometry_analysis_v2.json`, `261014_lidar_short_motion_geometry_tracking_v2`에 둔다.
case 번호 0을 정적 장면으로 가정하던 분석기의 companion 선택 오류를 수정했다.
초기 기하 비교 v1은 입력 준비 전 시작해 실패했으며 정상 추론에 합산하지 않는다.

## 검증과 인계

새 C++ 순위 검사 10개와 기존 lifecycle/velocity 검사 19개가 통과했다.
후단 순수 단위 검사 55개, 연결 audit 37개, 새 도구 4개, live 분석 3개,
LiDAR 분석·묶음 13개, production bundle 6개도 통과했다.
운영 ON·명시적 OFF·generic OFF의 실제 launch parameter 해석을 확인했고,
설치본의 detection→tracking→prediction→marker ROS 검사를 통과했다.
ROS fixture가 필요한 검사를 plain unittest로 실행한 초기 실패는 정상 검증에 합산하지 않는다.
별도 master에서 올바른 rostest 실행 후 XML의 1검사·0실패를 확인했다.
시나리오·원본·기존 실패 기록은 보존하고 시험 프로세스만 종료한다.
마지막 재시작에서 실제 51개·cut=-0.01·npackets override 없음·P를 다시 확인했다.

팀장님 planning 브랜치에는 아직 이 후속을 자동 merge하지 않았다. matching Git/submodule과
증분 빌드를 배포한 뒤 fresh stack으로 전체 주행을 확인한다. 기존 E24 ZIP과 가중치는
변경하지 않았으며 tracker 변경은 Git/submodule로 전달한다.

학습 Z 위치·높이 loss는 유지한다. 기본·거리별 AP는 XY 중심 거리이고 BEV IoU도
Z/높이를 사용하지 않는다. mASE에는 높이가 들어가지만 Z 위치는 들어가지 않는다.
높이 loss 제거가 XY 검출을 개선한다는 근거가 없어 이번 작업에 묶어 변경하지 않는다.
