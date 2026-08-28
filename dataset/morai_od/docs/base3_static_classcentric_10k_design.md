# Base3-Static 10K class-centric dataset.

## 이름과 목적.

- 정식 Dataset ID는 `morai_static_classcentric_base3_10k`다.
- 짧은 표기는 `Base3-Static 10K`, 실험 접두사는 `B3S`다.
- Base3의 연속 MGeo competition-route Ego sampling을 사용한다.
- 모든 배치 actor는 정적이며 Ego만 주행한다.
- velocity/temporal production dataset이 아니라 single-frame 3D OD 표현력의 통제
  실험군이다.

## 배경 가설.

- E65는 약 1,500장의 작은 정적 데이터로 학습됐지만 online rosbag Vehicle 성능이
  예상보다 강했다.
- Base2는 10K 동적·미션 중심 데이터였지만 Vehicle-positive camera sample이 1.76%뿐이고
  36개 scene에 집중됐다.
- Base2의 자연 분포 LRASPP는 offline Val Vehicle IoU 0.70에 도달했지만 online
  로터리·고주로 Vehicle을 거의 전부 Background로 예측했다.
- GT database sampling을 적용한 S1g가 online에서 크게 개선된 것도 실제 객체-positive
  관측 밀도가 병목이라는 근거다.
- 따라서 `동적 현실성`과 `객체 외형·거리·시점 관측량`을 분리하려면 E65 의도를 확대한
  정적 class-centric 10K가 필요하다.

## 재개 계약과 현재 상태.

- 이 campaign은 폐기하거나 후순위로 내린 계획이 아니다.
- 2026-08-28 생성기 구현, 7-batch offline audit, representative smoke와 spatial smoke를
  완료했다.
- 다음 재개 시에는 아래 순서로 진행한다.
  1. production Batch 001--007을 workspace에 생성하고 설치 전 audit한다.
  2. 10,002 frames를 capture하고 clip-safe split 및 artifact completeness를 검증한다.
  3. 전체 capture에서 camera positive-pixel·LiDAR visibility 분포를 다시 집계한다.
  4. LRASPP from-scratch를 먼저 학습한 뒤 online semantic audit gate를 적용한다.
  5. gate 통과 시에만 같은 semantic checkpoint로 painted dataset을 만들고
     single-frame PointPainting을 학습한다.
- rosbag 검토가 길어져도 이 순서를 임의로 건너뛰거나 Base2 추가 sampling 실험으로
  대체하지 않는다.

### Base2 balanced sampling 선행 실험의 결론.

- `base2-lraspp4class-balanced-clipaware-e30`은 E19에서 best mIoU `0.7441`을 기록했다.
- natural E22 대비 offline 변화는 다음과 같다.
  - 4-class mIoU `+0.0005`.
  - Pedestrian IoU `+0.0105`.
  - Vehicle IoU `+0.0004`.
  - Obstacle IoU `-0.0088`.
- online semantic audit에서는 로터리와 고주로 Vehicle point를 여전히 거의 전부
  Background로 분류했다.
- 같은 S1g PointPillars에 checkpoint만 교체한 rosbag 결과도 기존 production보다 크게
  낮았다.
  - 로터리 Vehicle center mAP/F1@1m: `0.091/0.301`.
  - 고주로 Vehicle center mAP/F1@1m: `0.090/0.339`.
  - 기존 production은 각각 `0.720/0.888`, `0.803/0.882`였다.
- 결론은 명확하다.
  - 기존 36개 Vehicle-positive scene을 더 자주 sampling하는 것만으로는 새로운 외형,
    위치, 방향, 배경, camera view를 만들 수 없다.
  - scene 수와 실제 객체-positive 관측 밀도를 늘리는 Base3-Static 10K가 다음 controlled
    experiment다.
  - balanced LRASPP는 production에 적용하지 않는다.

## Campaign 구성.

총 3,334 clips, 10,002 frames다. 한 clip의 세 frame은 같은 actor layout을 공유하므로
split도 반드시 clip 단위로 유지한다.

| Family | 한글 의미 | Clips | Frames | 비율 |
| --- | --- | ---: | ---: | ---: |
| `stopped_vehicle` | 정적 Vehicle 중심 | 1,167 | 3,501 | 35.00% |
| `static_obstacle` | 정적 Obstacle 중심 | 1,167 | 3,501 | 35.00% |
| `standing_pedestrian` | 정적 Pedestrian 중심 | 500 | 1,500 | 15.00% |
| `realistic_mixed_static` | 세 class 혼합 | 167 | 501 | 5.01% |
| `sparse_background` | route-wide 무객체 hard-negative | 333 | 999 | 9.99% |
| 합계 |  | 3,334 | 10,002 | 100.00% |

- Train/Val/Test는 family별 약 70/15/15로 나눈다.
- 7-batch 누적 결과는 Train 2,328, Val 503, Test 503 clips다.
- 같은 위치 자체를 split label로 만들지 않는다. 모든 split이 route 전역을 보되 같은
  actor layout의 세 frame만 같은 split에 묶는다.

## 캡처 계약.

- `rolling-grouped`를 사용한다.
- scene당 3 frames다.
- trigger 간 Ego 이동 거리는 3 m다.
- 8 frames를 83 ms 간격으로 저장하는 Base2 temporal 계약을 사용하지 않는다.
- 이 데이터의 목적은 velocity가 아니라 서로 다른 view의 single-frame detection이다.
- 실제 trigger timestamp와 Ego/Object sidecar는 기존 SaveSensorData runner가 그대로
  저장한다. 추후 정적 velocity=0 검증에도 사용할 수 있다.
- 2026-08-28 live audit에서 `rolling-grouped`가 sensor artifact만 저장하고 Base2의
  `dynamic_metadata`를 쓰지 않는 누락을 발견해 수정했다.
  - 각 trigger의 Ego pose·heading·raw velocity·acceleration을 저장한다.
  - `/Object_topic` actor pose·size·heading·raw velocity·acceleration과 3-frame finite
    difference derived velocity를 저장한다.
  - Ego-relative actor state와 consecutive Ego transform을 저장한다.
  - 주변 `/Ego_topic`, `/Object_topic`, `/clock`, camera/LiDAR header history는 실제
    topic availability를 명시하며, Capture Mode에서 발행되지 않은 stream은 합성하지 않는다.
  - 3-frame live smoke에서 Ego/Object callback offset은 약 2--6 ms였고, 정적 Vehicle
    derived speed는 0--0.000007 m/s였다.
  - clip metadata는 `scenario_static_zero`를 정적 canonical label로, fresh MORAI raw를
    동적 canonical source로, derived velocity를 `diagnostic_only`와 loss weight 0으로
    명시한다. 원본 audit 값은 보존하되 학습·handoff에는 하나의 canonical target만 노출한다.

## 공간·actor 정책.

### Ego.

- sparse global waypoint 자체를 균등 추출하지 않는다.
- Base3에서 검증된 MGeo directed-link 폐루프 2,184.61 m를 사용한다.
- family별 route slot과 slot 내부 jitter를 사용해 전체 대회 route에 균등하게 분포한다.
- batch별 phase offset을 달리해 7개 batch에서 위치가 반복되지 않게 한다.

### 공간 stratum.

- 고주로를 별도 과표집하지 않는다. MGeo 폐루프에서 고주로가 차지하는 길이와 같은
  `21.7%`를 유지하되 세 합법 차로 `6/8/10`에 exact interleaved schedule로 균등 분산한다.
- 10K 누적 고주로 clip/lane quota는 다음과 같다.
  - Vehicle: `253 clips`, `85/84/84`.
  - Obstacle: `253 clips`, `85/84/84`.
  - Mixed: `36 clips`, `12/12/12`.
  - Empty: `72 clips`, `24/24/24`.
- 로터리 Vehicle recall과 hard-negative를 보강하기 위해 Vehicle `20%`, Mixed `15%`,
  Empty `15%`를 명시 quota로 둔다.
- 로터리 내부 phase는 접근 `27%`, 진입·합류 `30%`, 순환 `35%`, 이탈 `8%`다.
- 이탈은 10% 이하로 제한한다. 후방으로 멀어진 차량보다 접근·진입·순환의
  Vehicle-positive를 우선하기 위한 결정이다.
- 나머지 route-general 표본은 로터리와 고주로 arc를 제외한 폐루프 길이에 균등 분포한다.
  따라서 명시 quota와 자연 sampling이 같은 구간을 이중 과표집하지 않는다.

### Vehicle-centric.

- clip당 3--5대다.
- personal launcher에서 자동 capture 가능하다고 검증된 Vehicle 43종을 균등 deck으로
  순환한다.
- actor는 Ego swept corridor 밖의 실제 MGeo lane-centre 후보에 배치한다.
- 중심선을 단순히 좌우 3.5--5 m 평행 이동한 좌표는 포장면을 보장하지 않으므로
  production에서 사용하지 않는다.
- 목표 lateral 좌표의 2.75 m 이내에 합법 MGeo 차로가 없으면 동일 차로의 안전한
  전·후방 queue로 대체한다.
- 종방향 slot은 -12, -5, 3, 11, 19 m이며 scene마다 섞는다.
- 대부분 도로 방향에 정렬하고 yaw jitter는 ±8 deg다.
- 반대 방향 주정차 appearance는 10%만 포함한다.
- 대형 버스·트럭은 차종별 보수적 footprint envelope를 사용한다. 중심점 clearance만으로
  안전하다고 판정하지 않는다.
- 고주로 최외곽 차로는 바깥 shoulder가 아니라 가운데 차로 방향의 포장면에 배치한다.
- 로터리 접근은 양측 view를 유지한다. 진입·순환·이탈은 MGeo arc를 따라가는 전방
  stopped queue로 구성한다.
- 로터리 접근·진입·순환 scene의 70%는 공식 NPC MGeo path에서 Ego 좌측에 있고 Ego
  swept route와 3.15 m 이상 이격된 정적 Vehicle snapshot을 한 대 포함한다.
- 고주로 Ego lane 6/8은 각각 lane 8/10의 합법 좌측 인접 차로 snapshot을 포함한다.
  좌측 차로가 없는 lane 10에는 이를 억지로 만들지 않는다.
- 좌측 snapshot은 동적 motion이나 sensor timing을 흉내 내는 것이 아니라 거리·yaw·가림·
  카메라 view 기하를 보강하는 static controlled sample이다.
- 전방 queue는 Ego가 세 frame 동안 약 9 m를 이동한 뒤에도 첫 차량까지 안전거리가
  남도록 배치한다.

### Obstacle-centric.

- clip당 5--8개다.
- 검증된 Obstacle 30종과 기존 subtype quota deck을 사용한다.
- box, barrel, barrier, bicycle/scooter, road sign, fixed roadside, road-surface를
  starvation 없이 순환한다.
- Ego swept corridor와 asset subtype별 최소 clearance를 유지한다.

### Pedestrian-centric.

- clip당 4--7명이다.
- launcher에서 안정적인 실제 person asset 22종을 사용한다.
- scene의 65%는 route 인접 crosswalk, 35%는 route-wide roadside context다.
- crosswalk/general 배치는 Train 앞부분에 한쪽 모드가 몰리지 않도록 exact interleaved
  schedule을 사용한다.

### Mixed와 hard-negative.

- Mixed는 Obstacle 1--3, Vehicle 1--2, Pedestrian 1--2를 함께 배치한다.
- hard-negative 333 clips은 actor 0개다.
- 첫 모델 이후 전체 주행 rosbag false positive를 수집해 중앙분리대·가드레일·기둥
  targeted hard-negative add-on으로 확장한다.
- Mixed는 세 class를 독립 배치한 뒤 끝내지 않는다. 차종·subtype별 footprint envelope와
  0.8 m margin으로 class 간 상호 겹침을 검사한다.

## Offline 검증.

MORAI를 실행하지 않은 상태에서 Batch 001과 7-batch 전체를 생성했다.

- 7개 primary manifest 전부 기존 static validator `pass`다.
- 신규 class-centric audit도 7/7 `pass`다.
- 총 scene ID, scenario stem, random seed가 각각 3,334개이며 중복은 0개다.
- 동일 seed/output 조건으로 Batch 001을 다시 생성했을 때 scenario JSON byte가 전부
  동일했다.
- 누적 runtime asset coverage는 Vehicle 43종, Pedestrian 22종, Obstacle 30종이다.

| 예상 capture view | Vehicle | Pedestrian | Obstacle |
| --- | ---: | ---: | ---: |
| Front | 10,695 | 6,543 | 20,432 |
| Left | 1,499 | 855 | 1,450 |
| Right | 1,599 | 1,253 | 1,739 |
| Rear | 607 | 355 | 88 |

- 이 view 표는 metadata pose와 3 m trigger를 이용한 offline 근사다.
- 같은 근사 기준 거리 분포는 다음과 같다.
  - Vehicle: 0--10 m 33.8%, 10--20 m 25.1%, 20--30 m 24.5%, 30 m+ 16.7%.
  - Pedestrian: 0--10 m 38.7%, 10--20 m 26.6%, 20--30 m 23.4%, 30 m+ 11.3%.
  - Obstacle: 0--10 m 22.2%, 10--20 m 31.3%, 20--30 m 29.1%, 30 m+ 17.3%.
- 실제 visible GT 수와 camera별 pixel 분포는 MORAI smoke/capture 이후 다시 audit한다.
- Obstacle rear view는 작지만 정적 장애물은 통과 전 전방 인지가 핵심이라 현 baseline에서는
  허용한다.

## Live 검증 상태.

2026-08-28 MRP live smoke를 수행했다.

- 최초 representative smoke는 5 clips 중 3개가 성공했다.
  - Vehicle은 대형 bus가 route yaw와 어긋나 Ego corridor를 침범했다.
  - Mixed는 Vehicle과 long obstacle collider가 겹쳐 일부 actor가 사라졌다.
- footprint·mutual-clearance 수정 뒤 같은 5 clips, 15 frames가 전부 성공했다.
- 로터리 4 phase와 고주로 3 lane을 강제한 spatial smoke 12 clips를 추가했다.
  - manifest를 `fixed_static`으로 구분해 Vehicle도 pre-capture 안정성 gate에 포함했다.
  - 고주로 바깥 shoulder와 로터리 roadside에서 사라지는 Vehicle을 이 gate가 발견했다.
  - 고주로는 도로 안쪽 방향 배치, 로터리 진입·순환·이탈은 MGeo arc 기반 same-lane
    stopped queue로 수정했다.
  - 이전 실패 고주로 lane 6, 로터리 circulation, entry, exit scene은 개별 재검증에서
    actor count, 0.5 s stability, 3-frame artifact completeness를 통과했다.
- 최종 spatial smoke는 `12/12 clips`, `36/36 frames`, 실패 0으로 통과했다.
  - Vehicle 7 clips은 로터리 4 phase와 고주로 3 lane을 각각 하나 이상 포함했다.
  - Obstacle 3 clips은 고주로 3 lane을 각각 포함했다.
  - 로터리 Mixed와 Empty도 각각 통과했다.
  - 4-camera BBox2D/BBox3D positive 파일은 각각 86개였고, LiDAR Instance/8Points GT는
    actor가 있는 33/36 frames에서 모두 존재했다. 나머지 3 frames는 의도한 Empty다.
- `rolling-grouped`의 실제 wall-time trigger rate는 약 `0.66--0.84 Hz`였다. 이는 3 m
  거리 trigger와 SaveSensorData 쓰기 시간의 결과이며 temporal 12 Hz 계약이 아니다.

### Production 첫 중단과 geometry 수정.

- 최초 production Batch 001은 9 scene 시도 중 5개 성공, 4개 실패했다.
- 실패 scene은 모두 Vehicle 중심 scene이며 기대 NPC `3/3/3/4대` 중 실제
  `/Object_topic`에는 `1/1/2/2대`만 남았다.
- ROS 전체 단절이 아니라 일부 정적 Vehicle만 사라진 결과였고, 네 scene이 연속으로
  실패했다.
- 원인은 route centerline을 좌우로 수학적으로 이동한 좌표가 실제 인접 차로·포장면인지
  보장하지 않는 placement 정책이었다. asset 블랙리스트 문제로 취급하지 않는다.
- 수정 뒤 offline Batch 001은 다음을 통과했다.
  - 기존 validator `477/477 scenes`, error/warning 0.
  - class-centric audit pass.
  - Vehicle 거리 추정: 0--10 m 16.6%, 10--20 m 26.5%, 20--30 m 30.4%,
    30 m+ 26.5%.
  - MGeo lane-snap 238 placements, safe same-lane fallback 330 placements.
  - roundabout legal-left 16 placements, highway legal-left 17 placements.
- 동일 차로 fallback에서 대형 Vehicle footprint 때문에 4/5번째 actor를 안전하게 놓을 수
  없으면 family 최소 계약인 3대까지만 유지한다. actor 수를 맞추려고 겹침을 허용하지 않는다.

## 후속 동적 sensor-domain 문제.

Base3-Static에서 지금 해결하는 항목과 후속 연구를 분리한다.

- 지금 해결하는 항목.
  - 로터리·합류부 좌측 Vehicle의 거리·yaw·가림·camera view 기하.
  - 실제 MGeo 차로 위의 안정적인 정적 actor placement.
- Base3-Static으로 해결하지 않는 항목.
  - ROS online 카메라와 LiDAR의 독립 주기·timestamp에 따른 sensor 비동기.
  - 한 LiDAR scan 내부 point acquisition time 차이에 따른 rolling-scan distortion.
  - Ego-only multi-sweep alignment에서 움직이는 NPC point가 여러 위치에 남는 ghosting.
  - 실제 합류 차량의 접근·이탈 motion sequence.
- 현재 MORAI 공식 camera 설정과 프로젝트 sensor JSON에는 exposure/shutter 기반 motion
  blur를 활성화했다는 근거가 없다. JPEG artifact와 위 temporal 문제를 camera motion
  blur라고 부르지 않는다.
- 후속 실험 순서는 다음과 같다.
  1. Base3-Static으로 single-frame geometry/appearance 병목을 먼저 분리한다.
  2. 실제 online rosbag에서 camera/LiDAR header 차이와 LiDAR scan 특성을 계측한다.
  3. rosbag-derived asynchronous training/fine-tuning A/B를 수행한다.
  4. multi-sweep이면 Ego alignment와 moving-object ghosting을 별도 ablation한다.

남은 production 전 검증은 다음과 같다.

1. 전체 capture 이후 실제 camera별 positive pixel과 LiDAR GT distance/view 분포를
   offline 근사치와 비교한다.

## Production Batch와 수집 runner.

- 현재 production source는 Queue5 동일 차선 역할 분리와 로터리 clearance 보강을
  반영한 campaign tag `queue5prod`, seed base `260829000`이다.
- 기존 `prod2`, `safety3` source·capture·MRP 설치 사본은 삭제하고 Batch 001--007을
  전부 새 scene ID로 재생성했다.
- primary는 총 `3,334 clips`, `10,002 frames`다.
- 각 batch는 기존 validator와 class-centric audit를 모두 통과했다.
- `run_base3_static_10k_capture.sh`가 다음을 담당한다.
  - 현재 batch만 MRP Scenario directory에 설치한다.
  - `rolling-grouped`, 3 m 간격, 3 frames로 capture한다.
  - batch가 끝나면 설치 사본을 `~/morai_scenario_archive/base3_static_10k`로 옮긴다.
  - `/tmp/morai_base3_static_10k_session_dir`와 session의 `current_capture.tsv`를 원자적으로
    갱신해 두 개의 read-only monitor가 batch 전환을 자동 추적하게 한다.
  - 중단된 batch는 `--resume` 시 기존 `summary.json`을 읽어 성공 scene을 보존하고
    실패·미실행 scene만 다시 수행한다. `console.log`도 덮어쓰지 않고 append한다.
- source scenario와 manifest는 workspace에 유지되므로 archive는 capture dataset의 일부가
  아니다.

### Prod2 중단 분석과 safety3 placement pilot.

2026-08-28 기존 `prod2` Batch 001 수집은 약 104 scenes에서 중단하고 원인을 다시
분석했다.

- 104 scenes 중 92개 성공, 12개 실패였다.
- 실패는 collision 7건, actor-count 5건이며 ROS 연결 실패는 없었다.
- 실패는 모두 `stopped_vehicle`에 집중됐다.
- 주요 원인은 다음 세 가지였다.
  - 14 m same-lane fallback을 차체 길이와 무관하게 사용해 bus/truck이 Ego와 충돌했다.
  - 로터리에서 centerline arc 거리는 멀지만 실제 2D 좌표가 다시 가까워지는 후보를
    허용했다.
  - `realistic_mixed_static`은 Vehicle 배치와 Ego spawn이 서로 다른 lead-distance 기준을
    사용했다.

수정 사항은 다음과 같다.

- Vehicle yaw 180도 flip을 제거했다.
- heading jitter를 차종별로 제한했다.
  - 승용/전기차 ±5도, SUV/MPV ±3도, bus/truck ±1도.
  - 명시적 legal-left path는 ±1도.
- same-lane 거리 deck을 차종별로 분리했다.
- 초기 Ego pose와 3개 capture pose 모두에 대해 실제 2D body clearance를 검사한다.
- 대형 Vehicle끼리는 footprint 합 외에 4 m pair margin을 요구한다.
- mixed scene이 Vehicle 배치와 Ego spawn에 동일한 preselected lead를 공유한다.
- 마지막 rolling-grouped trigger 직후 추가 50 ms 진행 없이 즉시 stop command로
  넘어간다.
- pilot용 audit는 전체 10K asset quota와 placement safety 검사를 분리한다.

동일 실패 seed의 target scene과 12-scene live smoke를 재검증했다.

- 기존 12 m 간격에서 사라진 `DEFAULT_BUS`는 16 m 간격에서 유지됐다.
- 12-scene smoke는 `12/12 scenes`, `36/36 frames`, 실패 0으로 통과했다.

그 뒤 `safety3` 120-scene placement pilot을 수행했다.

| 항목 | 결과 |
| --- | ---: |
| 요청/완료 scenes | 120 / 120 |
| 완료 frames | 360 / 360 |
| 실패 scenes | 0 |
| failure log rows | 0 |
| artifact quality failures | 0 |
| scene duration mean / p95 | 10.27 / 11.01 sec |
| scene당 timestamp groups | 항상 3 |
| capture 결과 용량 | 약 1.7 GB |

- Vehicle 108, Obstacle 3, Pedestrian 3, Mixed 3, Empty 3 scenes로 placement 안정성을
  Vehicle 쪽에 집중했다.
- spatial quota는 route-general 69, highway 26, roundabout 22, crosswalk 2,
  기타 1 scene이었다.
- roundabout 22 scenes는 approach 6, entry 6, circulation 8, exit 2다.
- highway lane 표본은 lane 6/8/10에 11/8/7 scenes로 분산됐다.
- LiDAR `Instance_8Points` GT는 positive 351/351 frames에서 모두 비어 있지 않았다.
- 나머지 9 frames는 의도한 Empty 3 scenes × 3 frames다.
- LiDAR GT row는 총 1,398개이며 Vehicle 계열, Pedestrian, Obstacle이 모두 존재했다.

기존 `prod2` 10K source batch는 위 수정 이전 생성물이므로 다시 사용하지 않는다.
이번 검증에서는 최종 10K batch를 새로 만들지 않았다. 다음 production batch 생성 시
새 campaign tag와 seed를 사용해 수정본으로 001--007을 전부 재생성한다.

### Safety3 Batch 002 중단 시점 측방 배치 감사.

2026-08-28 고주로의 좌측-only 명시 배치가 전체 같은 방향 차로 기하를 충분히
포함하지 못한다는 판단으로 수집을 중단하고, Batch 002의 완료분을 추가 감사했다.

| 항목 | 결과 |
| --- | ---: |
| 시도/완료 scenes | 128 / 128 |
| 완료 frames | 384 |
| 실패 scenes | 0 |
| route-general / highway / roundabout scenes | 74 / 28 / 26 |
| highway lane 6 / 8 / 10 scenes | 10 / 9 / 9 |
| scene당 Vehicle 평균 | 3.88대 |
| capture 시 Vehicle 거리 p50 / p95 | 27.52 / 65.38 m |
| Ego state age p50 / p95 | 10.3 / 28.2 ms |
| Object state age p50 / p95 | 10.2 / 29.5 ms |

생성 Vehicle 497대의 geometry role은 다음과 같았다.

| geometry role | Vehicle | 비율 |
| --- | ---: | ---: |
| `same_lane_safe_fallback` | 469 | 94.4% |
| `highway_left_adjacent_lane_8` | 9 | 1.8% |
| `highway_left_adjacent_lane_10` | 8 | 1.6% |
| `roundabout_left_legal_path` | 11 | 2.2% |

- 3개 frame의 실제 Ego-relative 관측 1,491건은 front 1,149, rear 328, left 14,
  right 0이었다.
- legal-left actor도 종방향 offset이 큰 경우 front sector로 분류되므로 geometry role과
  카메라 sector는 같은 개념이 아니다.
- 캡처 안정성과 class/asset 순환은 양호하지만, 측방 기하 보강량은 single-frame
  일반화 목표에 비해 부족하다.

현재 production 설정의 정확한 적용 범위는 다음과 같다.

- 일반 `route_general`의 Vehicle은 `same_lane_fallback_only=true`이므로 임의 측방
  배치를 수행하지 않는다. 왼쪽·오른쪽 랜덤 배치가 숨어 있는 구조도 아니다.
- 명시적 측방 Vehicle은 고주로 legal-left helper와 로터리 공식 NPC path helper에서만
  생성된다.
- 로터리 helper는 Ego pose를 단순 평행 이동하지 않는다. 공식 로터리 NPC MGeo path의
  위치와 접선 yaw를 사용하므로 합류부의 비평행 기하를 유지한다.

다음 생성기 수정 원칙은 다음과 같다.

- 고주로는 lane 6/8/10 중 Ego lane을 제외한 모든 같은 방향 lane을 후보로 열거한다.
  - 한 lane 차이는 높은 quota, 두 lane 차이는 낮은 quota로 둔다.
  - 가운데 lane 8은 lane 6/10을 균형 있게 배정한다.
  - target lane 안에서 종방향 위치를 먼저 바꿔 안전 후보를 찾고, 즉시 same-lane으로
    fallback하지 않는다.
- 일반 도로는 무작정 좌우 offset을 다시 활성화하지 않는다.
  - 이번 route-general 74 scenes의 target MGeo link 중 명시적 lane-change destination이
    있는 곳은 right-only 5곳뿐이고, 69곳은 topology상 이웃 lane 정보가 없었다.
  - 이웃 link가 존재할 때에는 같은 진행 방향, 일정한 횡거리, 충분한 평행 overlap,
    Ego route와의 분리, actor body clearance를 모두 검증한 경우에만 측방 배치를 허용한다.
  - topology 정보가 없는 곳은 geometric nearest-lane 하나만으로 추측하지 않고 현재의
    same-lane queue를 유지한다.
- 위 변경은 중단된 safety3 batch에 소급되지 않는다. generator test와 100--120 scene
  pilot을 다시 통과한 새 campaign으로 001--007을 재생성한다.

### Side6 측방 기하 재설계와 120-scene live pilot.

위 원칙을 `side6` 생성기에 실제로 반영했다.

- 고주로.
  - Ego lane 6/8/10 중 Ego가 없는 나머지 같은 방향 lane을 모두 후보로 사용한다.
  - 외곽 lane에서는 인접 lane 70%, 두 lane 떨어진 lane 30%를 우선한다.
  - 가운데 lane에서는 좌우 lane을 균형 있게 선택한다.
  - scene 일부에는 두 non-Ego lane을 동시에 포함한다.
- 로터리.
  - 접근·진입·순환에서 공식 NPC MGeo path를 이용해 좌측 Vehicle을 0/1/2대로
    분산한다.
  - 목표 quota는 30%/50%/20%에 가깝게 두되, 차체 간격을 만족하지 못하면 안전하게
    1대 또는 0대로 fallback한다.
  - 이탈 phase의 낮은 quota와 same-lane 표본은 유지한다.
- 일반 도로.
  - 임의 횡방향 offset은 사용하지 않는다.
  - MGeo link가 명시적인 lane-change destination을 제공하고, 진행 방향·도로 여유·차체
    간격 검사를 모두 통과한 경우에만 해당 link centerline에 Vehicle을 배치한다.
  - topology가 없는 곳은 기존 same-lane queue를 유지한다.

최종 static audit와 live pilot 결과는 다음과 같다.

| 항목 | 결과 |
| --- | ---: |
| static validation | 120 scenes, error 0, warning 0 |
| live 요청/완료 scenes | 120 / 120 |
| 완료 frames | 360 / 360 |
| 최종 실패 scenes | 0 |
| 최종 실패율 | 0% |
| scene duration mean / p50 / p95 | 10.20 / 10.04 / 10.84 sec |
| route-general / highway / roundabout | 69 / 26 / 22 scenes |
| highway lane 6 / 8 / 10 | 11 / 8 / 7 scenes |
| Ego state age p50 / p95 | 10.6 / 28.7 ms |
| Object state age p50 / p95 | 11.1 / 30.0 ms |

- 최초 실행에서는 `dsv001--003`이 MORAI ROS subscriber 일시 단절로 실패했다.
  같은 run을 resume한 뒤 세 scene 모두 정상 완료됐으며, placement·collision·artifact
  quality에 의한 최종 실패는 없었다.
- 생성 Vehicle 438대 중 same-lane은 384대, 실제 다른 lane/path를 사용한 Vehicle은
  54대다. 측방 기하 비율은 `12.3%`로 중단 Batch 002의 `5.6%`보다 약 2.2배다.
- 다른 lane/path Vehicle 54대의 구성은 highway 28대, roundabout 15대,
  route-general explicit topology 11대다.
- 3개 frame의 실제 Vehicle 관측 1,314건은 front 986, rear 299, left 19,
  right 10이다. 구버전의 right 0은 제거됐다.
- 관측 sector는 종방향 offset의 영향을 받으므로 다른 lane actor가 front/rear로도
  분류된다. 측방 다양성의 주 지표는 `geometry_role`이고 sector는 보조 지표다.
- 전체 Vehicle 거리 p50/p95는 26.33/58.02 m다. same-lane p05는 11.84 m이고,
  다른 lane의 짧은 center distance는 차로 횡분리 상태에서만 허용됐다.
- live pilot에서 collision은 0건이므로 앞차 안전거리 보강과 다차로 배치가 서로
  충돌하지 않았다.

결론적으로 중단된 safety3 Batch 001--002를 최종 데이터로 이어 쓰지 않는다. 다음
Base3-Static 수집은 `side6` 이후 generator로 새 scene ID와 seed를 사용해 001--007을
전부 재생성한다.

### Queue4 동일 차선 역할 분리와 LiDAR GT evidence 정책.

`side6` pilot의 실제 LiDAR를 다시 감사한 결과, 동일 차선 전방 queue가 학습 라벨
노이즈의 주 원인이었다.

- Vehicle scene의 79.1%에 동일 차선 전방 차량이 2대 이상 있었다.
- 첫 번째 전방 차량 152건은 zero-point 0%, point p50 96이었다.
- 두 번째 전방 차량 74건은 zero-point 45.9%, point p50 2였다.
- 세 번째 전방 차량 23건은 zero-point 65.2%, point p50 0이었다.
- Base2 train/val Vehicle GT 2,022건은 minimum 8 points였으므로 Base3의 희소 GT는
  모델 난이도 자체보다 생성 배치가 만든 불필요한 label noise로 판단했다.

따라서 Vehicle scene의 역할을 다음처럼 분리한다.

- 동일 차선 전방은 정확히 1대다.
- 동일 차선 후방은 0--1대이며 scene 전체에서 50:50으로 interleave한다.
- 고주로·로터리·명시적 MGeo topology의 옆 차선/path Vehicle은 위 제한과 독립이다.
- 옆 차선 후보가 body-clearance 검사를 통과하지 못하면 제거한다. 실패한 옆 차선
  actor를 두 번째 전방 차량으로 바꾸지 않는다.
- 고주로 한 대 측방 표본은 전방 60%, 후방 40% offset deck을 사용한다. 두 대면
  front/rear 하나씩 두되 lane assignment를 번갈아 배치한다.

거리 정합 감사 중 두 가지 generator 오류도 수정했다.

- 역할별 slot을 사용하면서 구형 slot key만 확인해 preselected Ego lead가 actor
  배치 전에 전달되지 않던 문제를 수정했다.
- sparse MGeo vertex에 Ego pose를 양자화하던 `route_pose_before`를 metric arc
  interpolation으로 변경했다.
- source pilot의 같은 차선 전방 목표 오차 p50은 약 14.6 m에서 0.15 m로 감소했다.
- 수정 후 초기/중간/마지막 capture의 실제 같은 차선 전방 거리 p50은 pilot 초반
  기준 약 21.0/18.0/15.3 m였다.

LiDAR GT는 raw MORAI 파일을 보존하되 학습/evaluation info 생성 시 evidence를
명시적으로 기록한다.

- `num_points_in_gt == 0`: GT에서 제거한다.
- `1 <= num_points_in_gt <= 4`: 정상 positive에서 제외한다.
- `num_points_in_gt >= 5`: 첫 Base3 normal-positive 기준이다.
- OpenPCDet `CustomDataset`의 `MIN_GT_POINTS`는 opt-in이며 기본값 0이라 기존 Base2와
  production 모델의 재현성에는 영향을 주지 않는다.
- Base3 info 생성 때 `MIN_GT_POINTS: 5`와 `num_points_in_gt`를 함께 사용한다.
- 실제 분포는 `audit_morai_lidar_gt_visibility.py`로 class·geometry role별 감사한다.

#### Queue4 live pilot과 Queue5 최종 보강.

Queue4 120-scene pilot을 MORAI `SaveSensorData`로 끝까지 실행했다.

- 120 scene 중 119 scene, 357 frame을 정상 저장했다. 실패율은 0.83%다.
- scene 처리시간 p50/p95는 10.05/10.83초, sequence rate p50/p05는
  0.81/0.70 Hz였다.
- Ego state age p50/p95는 11.2/28.2 ms, Object state age는
  11.7/30.5 ms였다.
- 같은 차선 전방 차량은 첫 frame 기준 110건이며 거리 min/p50/p90/max는
  16.84/21.92/29.91/30.28 m였다.
- 명시적 side actor 실제 관측은 159건이며 front-left 72, front-right 52,
  rear-left 20, rear-right 15건이었다. 동일 차선 queue 제한이 대각선 표본을
  줄이지 않았음을 확인했다.

30 m 이내 LiDAR GT 649개를 실제 point로 감사한 결과는 다음과 같다.

- 621개(95.7%)가 5 point 이상 normal-positive였다.
- 21개는 1--4 point sparse GT, 7개는 zero-point GT였다.
- Vehicle은 535개 중 533개(99.63%)가 normal-positive였고 zero/sparse는 각각
  1개뿐이었다.
- 동일 차선 전방 306건의 point p50은 61, 후방 112건은 30이었다.
- pilot의 Pedestrian·Obstacle은 각 3 scene뿐이므로 해당 class 비율은 production
  분포 결론이 아니라 evidence filter 동작 확인용이다.

유일한 실패는 로터리 진입 scene `dsv102`의 측방 SUV가 route에서 5.12 m밖에
떨어지지 않아 발생한 collision이었다. Queue5에서 로터리 side actor의 최소 route
clearance를 7.0 m로 올렸다. 같은 seed의 actor clearance는 13.90 m가 됐고, 해당
scene을 다시 live capture해 3/3 frame, collision 0건, 모든 Vehicle GT 20 point
이상을 확인했다.

따라서 최종 generator 기준은 Queue5다. 전체 120 scene을 다시 반복하지는 않았지만,
Queue4 full pilot과 유일 실패 seed의 Queue5 live 재검증을 함께 통과한 상태다. 실제
10K Batch 001--007은 이 기준으로 새 ID와 seed를 생성한다.

2026-08-28 `queue5prod` Batch 001--007을 생성했다.

- Batch 001--002는 각각 477 clips, 1,431 frames다.
- Batch 003--007은 각각 476 clips, 1,428 frames다.
- 전체는 3,334 clips, 10,002 frames다.
- 7개 batch 모두 capture-set validator의 errors/warnings가 0이고 class-centric
  spatial audit를 통과했다.
- 각 batch의 runtime asset coverage는 Vehicle 43종, Pedestrian 22종,
  Obstacle 30종이다.

### Queue5 부분 수집 LiDAR GT 감사와 다음 개편 계획.

`queue5prod` Batch 001은 477 scene 중 377 scene을 시도한 시점에 수집을 중단하고
Vehicle·Pedestrian·Obstacle의 실제 LiDAR evidence를 다시 감사했다. 이 결과는 완성
dataset이 아니라 다음 generator 개편을 위한 진단 자료다.

- 완료 373 scene, 실패 4 scene, 완료 1,119 frames다.
- 30 m 이내 실제 GT box는 Vehicle 819, Pedestrian 604, Obstacle 2,751개다.
- point 수는 MORAI `LIDAR_5/Instance.txt` box 안에 실제 XYZI point가 몇 개 있는지
  계산했다.

| Class | 전체 GT | 0 point | 1--4 point | 5 point 이상 | 5 point 미만 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Vehicle | 819 | 1 | 1 | 817 | 0.2% |
| Pedestrian | 604 | 10 | 70 | 524 | 13.2% |
| Obstacle | 2,751 | 266 | 627 | 1,858 | 32.5% |
| 전체 | 4,174 | 277 | 698 | 3,199 | 23.4% |

Vehicle은 같은 차선 queue 제한과 명시적 side-path 정책을 적용한 뒤 LiDAR evidence가
충분하다. 남은 Vehicle 후속 수정은 다음과 같다.

- `same_lane_primary_front`의 실제 종방향 부호가 항상 양수인지 생성·감사 단계에서
  invariant로 검사한다. 실패한 측방 placement가 전방 역할 이름을 가진 후방 slot으로
  대체되지 않게 한다.
- 같은 차선 전방은 최대 1대, 후방은 0--1대를 유지한다.
- 후방 차량은 capture가 진행되며 더 멀어지는 점을 고려해 subtype별 시작 거리를
  조금 당기되, Ego 차체가 LiDAR를 완전히 가리는 초근거리는 피한다.
- 옆 차선의 좌전방·우전방·좌후방·우후방 Vehicle은 같은 차선 제한과 독립적으로
  유지한다.
- 고주로는 실제 양쪽 인접 차로, 로터리는 실제 circulating path, 일반 도로는 명시적
  MGeo topology가 있을 때만 측방 Vehicle을 둔다.
- Vehicle yaw는 실제 lane/path 접선을 사용한다. 임의 180도 반전은 넣지 않고 실제
  반대 방향 차로에 배치한 경우만 반대 방향 관측으로 인정한다.

Pedestrian은 asset보다 방향·거리 분포가 우선 문제다.

- standing Pedestrian이 거의 도로 진행 방향 하나로 정렬된 현 상태를 폐기한다.
- route-relative 전방, 후방, 좌횡단, 우횡단 방향과 작은 yaw jitter를 혼합한다.
- 횡단보도에서는 횡단 방향을 우선하고, route-wide standing family에서는 네 방향을
  비교적 고르게 사용한다.
- 5 point 미만 13.2%를 줄이기 위해 근·중거리 비중을 높이고 과도한 원거리·가림
  배치를 줄인다.
- 특정 asset이 한 scene에서 빠졌더라도 다른 scene의 정상 spawn evidence가 있으면
  자산 고유 실패로 단정하지 않는다. ROS/MORAI 재시도 뒤 reserve로 보충한다.

Obstacle은 MORAI `objectList` 컨테이너와 대회 OD ontology를 분리한다. 공식 샘플
시나리오의 정적 장애물은 `40100001 CargoBox` 한 종류다. 대회가 LiDAR로 거의 관측할
수 없는 노면·고정 시설물을 회피 장애물로 낼 가능성은 낮다고 판단하고, 자동 positive
pool을 30종에서 18종으로 줄인다.

다음 12종은 catalog provenance만 유지하고 자동 배치와 Obstacle positive quota에서는
제외한다.

- `Road_Sign_Mark`: 관측 46/46개가 0 point다.
- `hole-cover`: 97.6%가 0 point이며 5 point 이상 관측은 0개다.
- `fr-streetlight-2`: 검출 가능 여부와 무관하게 고정 도로시설이다.
- `KR_Sign_SL_30`, `40`, `50`, `60`, `70`, `80`, `90`, `100`, `110`: 고정 속도
  표지판이며 이를 positive로 학습하면 도로시설 오검출을 강화할 수 있다.

유지하는 18종과 첫 quota는 다음과 같다.

| 그룹 | Asset | Obstacle 내부 비중 |
| --- | --- | ---: |
| box | `CargoBox`, `WoodBox` | 25% |
| barrel/drum | `PE_Drum`, `TrafficBarrel`, `RedBarrel`, `YellowBarrel` | 25% |
| large barrier | `concrete-barrier`, `Concrete_Barricade01`, `Concrete_Barricade02`, `Steel_Barricade`, `PE_Firewall_Orange`, `PE_Firewall_White` | 35% |
| small barrier | `Bollard01`, `Bollard04`, `Lane_Separation_Bar` | 10% |
| unoccupied two-wheeler | `Bicycle_city`, `Bicycle_mountain`, `Scooter_02` | 5% |

- 공식 샘플인 `CargoBox`는 Obstacle placement의 약 15%를 우선 목표로 한다.
- Bollard와 lane bar는 주로 10 m 이내, two-wheeler는 15 m 이내, box와 barrel은
  20 m 이내를 중심으로 둔다.
- 대형 barrier만 20--30 m stress 표본을 충분히 유지한다.
- box와 barrel은 다양한 yaw, 긴 barrier는 평행·대각·수직 yaw를 혼합한다.
- 현재 차로 전방뿐 아니라 회피할 인접 차로, 좌우 대각선, 추월 중 근거리 측방도
  중요한 positive다. 주행 가능 영역 밖 도로변·원거리 후방만 낮은 비율로 둔다.
- `PE_Firewall_White` 이동 0.124 m와 `Steel_Barricade`의 특정 지형 spawn 이탈은 다른
  scene에서 정상 evidence가 충분하므로 자산 전체 제거 사유가 아니다. stability gate로
  clip만 차단한다.

Obstacle relative-position 첫 목표는 현재 차로 전방 35%, 인접 차로·회피 경로 전방
25%, 좌우 대각선 20%, 근거리 측·후측방 10%, 비충돌 도로변 context 10%다. 긴 barrier
yaw는 수직 40%, 대각선 35%, 평행 25%를 첫 audit 목표로 사용한다.

### Mission-weighted dataset과 evaluation 원칙.

Base3-Static의 목적은 모든 MORAI asset을 모든 방향에서 동일하게 맞히는 범용 benchmark가
아니라 대회 Planning에 필요한 객체를 안정적으로 제공하는 것이다. 다만 미션 좌표에
과적합하지 않도록 route-wide background와 기하 기반 ROI를 함께 유지한다.

다음 10K 재생성의 첫 family 비율은 Vehicle 40%, Obstacle 30%, Pedestrian 15%,
empty/hard-negative 15%다. 현재 `queue5prod` 35/35/20/10 source에는 소급 적용하지 않고
새 campaign ID와 seed로 생성한다.

- Vehicle은 로터리 접근·순환, 고주로 합류·추월, GPS 음영 전방·인접 차로를 우선한다.
- Pedestrian은 횡단보도, 가림 뒤 진입, Ego corridor와 충돌 가능한 전·측방을 우선한다.
- Obstacle은 현재 차로와 가능한 회피 차로의 전방·대각·측방을 우선한다.
- empty/hard-negative는 대회 route 전역의 중앙분리대, 가드레일, 기둥, 고정 표지판
  오검출을 줄이는 역할이다.

평가는 다음 네 slice를 함께 보고 production 선택은 `Mission-relevant`를 주 지표로 한다.

- `Global`: 기존 전체 센서 범위 AP·ATE·ASE·AOE·AVE다.
- `Mission-relevant`: lane relation, 상대 종횡방향, 거리와 가능한 Ego corridor로 고른
  객체다. 특정 scenario ID나 절대 좌표를 하드코딩하지 않는다.
- `Peripheral`: 원거리·후방·도로 밖 객체 진단 지표다.
- `Sparse`: LiDAR point가 적은 별도 stress 지표다.

로터리는 좌측 접근·순환 Vehicle, 고주로는 같은 차로 전방과 양쪽 인접 차로 Vehicle,
횡단보도는 corridor로 진입하는 Pedestrian, 정적 장애물과 GPS 음영은 현재·회피 차로의
Vehicle/Obstacle을 주요 ROI로 삼는다.

### Loss 변경 경계와 다음 검증 순서.

첫 Base3 개편에서는 dataset 효과와 loss 효과를 섞지 않는다. 필수 변경은 학습 target
quality mask뿐이다.

- 0 point GT는 classification·center·size·yaw loss 전체에서 제외한다.
- 1--4 point GT도 첫 normal-positive 학습에서는 제외한다.
- 5 point 이상 GT만 기본 positive로 사용한다.
- raw MORAI GT는 보존하고 OpenPCDet info 생성 시 `MIN_GT_POINTS: 5`로 mask한다.
- `WoodBox` 대표 zero-point 표본에서 실제 point가 GT 경계보다 약 2--4 mm 밖에 있던
  수치 사례가 있어 point-count 판정에만 약 2 cm tolerance를 줄지 A/B한다. GT box를
  임의로 키우지는 않는다.

첫 baseline에서는 class loss weight, GT sampling 비율, velocity loss, yaw loss 공식을
바꾸지 않는다. Vehicle과 긴 barrier, bicycle, 직사각형 box의 yaw는 유지해야 한다.
barrel·drum·bollard처럼 회전 대칭인 자산과 near-symmetric Pedestrian의 yaw weight 또는
mask는 새 baseline의 subtype별 mAOE와 loss gradient를 확인한 뒤에만 별도 A/B한다.

구현·검증 순서는 다음이다.

1. Vehicle role invariant, Pedestrian yaw deck, Obstacle 18종 pool·quota·거리·yaw 정책을
   generator와 static audit에 구현한다.
2. family별 smoke capture로 실제 spawn·방향·거리·artifact를 확인한다.
3. 100--120 scene live pilot을 실행해 실패율 한 자릿수, 가능하면 5% 이하를 확인한다.
4. 30 m 이내 point audit에서 Vehicle 5 point 미만 1% 이하, Pedestrian 5--8% 이하,
   Obstacle 10--15% 이하를 첫 목표로 삼는다.
5. 학습 info의 quality mask 적용 후 실제 학습 target에는 5 point 미만 GT가 0개인지
   확인한다.
6. Camera·LiDAR 정성 표본과 mission/global/peripheral/sparse 분포를 확인한 뒤에만 기존
   중단 capture와 source를 정리하고 Batch 001--007을 새로 생성한다.

이 절의 Vehicle 후속 미세조정, Pedestrian 방향 다양화, Obstacle pool 축소, family 비율
변경과 mission evaluator는 아직 다음 generator campaign의 계획이다. `queue5prod` 부분
수집 결과를 완성 dataset으로 오인하지 않는다.

## 모델 실험 순서.

1. 현재 Base2 natural LRASPP.
2. Base2 clip-aware balanced LRASPP.
3. Base3-Static 10K LRASPP from scratch.
4. 같은 Base3-Static split로 7D single-frame PointPainting from scratch.
5. offline Val뿐 아니라 새 로터리·고주로·Pedestrian·Obstacle rosbag online replay
   benchmark를 비교한다.
6. Static 10K가 online Vehicle/Pedestrian/Obstacle을 모두 개선하면 Static 60--70%,
   mission dynamic 20--30%, hard-negative 10% hybrid를 별도 실험한다.

- Static 10K는 velocity regression이나 temporal fusion의 대체물이 아니다.
- single-frame 표현력 baseline을 먼저 확립한 뒤 temporal/velocity dataset과 모델을
  분리해 결합한다.
