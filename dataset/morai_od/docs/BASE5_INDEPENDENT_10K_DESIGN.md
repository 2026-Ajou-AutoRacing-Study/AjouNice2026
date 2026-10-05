# Base5 독립 10K — single-frame OD 수집 설계

2026-10-05. **PLAN / 생성기 v2 구현 전.** 이번 설계는 기존 Base5 보강용 v1을
대체하는 다음 목표다. 현재 실행 코드/config가 아래 quota를 구현했다는 뜻은 아니다.

## 1. 목적과 바뀌지 않는 계약

- Base3와 섞지 않고 새 Base5만으로 학습 가능한 독립 3-class 데이터셋을 만든다.
  E25 fine-tuning은 필수가 아니다. 완전 random-init과 외부 pretrained 초기화는
  학습 실험에서 별도로 기록한다. 기존 E25 production은 새 모델의 rosbag 검증 전 유지한다.
- Temporal modeling 및 detector velocity head는 하지 않는다. 속도는 tracker가 추정한다.
- 실제 움직이는 사람·동물·탑승자·유모차 등의 동작 중 외관을 독립 frame으로 학습한다.
  움직임에 따라 외관이 달라지는 문제를 정지 template만으로 대체하지 않는다.
- MR만 사용하고 scale=1, 동기 캡처 LiDAR Instance geometric GT를 사용한다.
  센서 GT에 Object_topic pivot 보정을 다시 적용하지 않는다.
- 10,000은 시도 횟수가 아니라 quality gate를 통과한 최종 frame 수다.
  기본은 독립 배치 1 scene × 1 capture다. 동작 시점의 연속 복사로 수량을 부풀리지 않는다.
- Raw에는 세부 종류, component, visibility, motion 상태를 보존한다.
  전후방100m raw 수집과 ±30m production 모델 교체는 별개의 작업이다.

## 2. Base3/Base4 대비

| 항목 | Base3-Static 최종 이력 | Base4 temporal 이력 | 새 Base5 목표 |
|---|---|---|---|
| 주 목적 | 범용 single-frame OD | temporal/velocity 실험 | 독립 single-frame OD, 근접·원거리·동작 외관 |
| 기본 규모 | 3,334 scene ×3 =10,002 frame | 여러 sequence pilot | 10,000 accepted 독립 배치 |
| 기본 family | 차량40%, 장애물30%, 정지ped15%, sparse15% | 고주로 Vehicle 중심 | 차량40%, 장애물30%, ped/동적관련25%, 배경5% |
| 동작 보강 | 이후 움직이는ped900장 추가, 총10,902장 | 연속 이동/sweep | 동작 상태에서 독립 캡처, velocity target 없음 |
| 공간/자세 | route-wide +로터리/고주로 quota | Ego scene-constant 횡 offset | Ego 횡 offset+yaw, NPC 경로 정렬 유지 |
| 검증 | 같은scene 3장은 동일split | sequence split | 위치/배치/동작 variant 그룹 split |

Base3 로터리는341scene(1,023frame, 기본scene의10.2%)였다. 차량 family의20%,
sparse의15%를 명시적으로 배정했다. 새 Base5는 로터리 차량700장을 별도 보호한다.
Base3 일반 정적 차량 코드에는 subtype별 heading jitter가 있었다. 이것을 새 Base5의
경로 주행 NPC에 그대로 복사하지 않는다. Base4 고주로 NPC는 경로상 위치/yaw를 사용했고
임의 횡 offset은 Ego 경로에만 적용했다.

근거: [Base3 최종 감사](legacy/base3_static_10k_final_capture_audit.md),
[Base4 횡 offset 결과](260922_base4_velocity_hardcase_v2_results.md).

## 3. 하나의 고정 quota: 범용60 / 미션35 / 배경5

각 행은 primary target family이며 다른 class의 보조 객체가 함께 있을 수 있다.
장면 수와 class별 GT 수를 혼동하지 않는다. 아래 표 하나를 상위 quota의 기준으로 한다.

| 주 대상/공간 | 범용 랜덤 | 미션 조건부 | 합계 |
|---|---:|---:|---:|
| 고주로 차량 | 1,800 | 1,200 | 3,000 |
| 로터리 차량 | 200 | 500 | 700 |
| 기타 도로·정적 차량 | 200 | 100 | 300 |
| 실체 장애물 | 1,800 | 1,200 | 3,000 |
| 보행자·동적/복합 객체 | 2,000 | 500 | 2,500 |
| 명시적 빈 배경 | 0 | 0 | 500 |
| 합계 | 6,000 | 3,500 | 10,000 |

범용은 맵과 무관한 허공 배치가 아니다. 허용 도로/보도/지면 위에서 asset·거리·방위·
Ego 자세를 넓게 바꾼다. 미션 조건부는 동일샘플 복제가 아니라 접근/회피/합류 등 실제
관측 상황을 더 자주 뽑는 것이다. 샘플 위치/차종/간격/속도 하나에 고정하지 않는다.

## 4. 차량4,000장

### 4.1 고주로3,000: 거리·방향·도로구조를 별개 축으로 관리

거리 quota는 최종 Ego frame의 geometric center XY 거리 기준이다.

| 거리 | 전방 longitudinal 부호 | 후방 longitudinal 부호 | 합계 |
|---|---:|---:|---:|
| 0–30m | 525 | 525 | 1,050 |
| 30–60m | 525 | 525 | 1,050 |
| 60–100m | 450 | 450 | 900 |

표의 전/후는 longitudinal 부호이며 측방도 포함한다. 특히0–30m의 전방525/후방525
중 각150장은 |longitudinal|보다 |lateral|이 큰 측방·후측방 관측 목표로 둔다.
이 구분과 별도로 same-lane50% / legal-adjacent-lane50%를 목표로 한다.
모든 축의 조합이 모든 도로 위치에서 가능한 것은 아니므로 불가능한 cell은 명시하고
다른 유효 위치에서 보충한다. 조용한 front fallback으로 quota 성공 처리하지 않는다.

도로 구조: 합류600 / 본선 순항1,500 / 차로 축소·병목600 / 출구·톨게이트 접근300.
100m 후보는 경로 길이와 실제 NPC 주행 가능 구간이 있는 위치에만 배치한다.
샘플 despawn 이후를 기본 배치 구간으로 삼지 않는다. 실제 시나리오 destination과
MGeo 경로를 대조해 valid bounds를 저장한다. 곡선 arc100m와 XY100m를 혼동하지 않는다.

일반43차종을 사용한다. 본선 고주로 RandomMode는 미정이며 사용 시MiddleSize라는
주최측 답변을 반영해, 공식 subtype/optionList로 식별한 MiddleSize 후보70%, 나머지
일반 차종30%를 초기안으로 둔다. 문자열의 Sedan/SUV만 보고 MiddleSize를 추측하지 않는다.
동일군 안에서는 DataID 균형 deck을 사용하며 bus/truck 등의 후보가 삭제되지는 않는다.
16개 정적 OBJ 차량은 아래 정적 회피/일반 차량 쪽에 할당하고 정상 주행 NPC로 쓰지 않는다.

### 4.2 로터리700

접근190 / 진입·합류210 / 순환245 / 이탈55장. 500장은 미션 조건부,200장은 넓은
거리/asset 배치다. 거리0–10m175 /10–20m280 /20–40m245를 초기안으로 둔다.
현재 경로와 교차 경로의 진입차·좌측 순환차·앞차·통과 직후 차량을 포함한다.
500 미션장 중 단일차300 / 서로 분리된2차160 /3차40을 목표로 한다.
NPC position/yaw는 각자의 실제 로터리 경로를 사용한다. Ego 방향에 억지로 평행하게
놓지 않는다. 본선 고정 차종은 비공개이므로 K5만 대량 반복하지 않고43차종 coverage를 유지한다.

### 4.3 기타/정적 차량300

일반 도로 경로정렬150 / 정적 차량 회피150. 후자는 일반차량과 검증된 OBJ 차량16종을
균형 있게 포함한다. 정적 주차 차량은 도로 상대0°70%, ±15–45°20%, 기타 가능한
가로/비스듬한 배치10%를 제안하되 충돌/지면검사한다. 문 열린 G80은 보류이며
문 닫힌 일반 G80은 포함한다. 미션 보조 정적 차량도 별도 asset coverage로 집계한다.

## 5. 실체 장애물3,000장

일반 pool19종: CargoBox, WoodBox, RedBarrel, YellowBarrel, TrafficBarrel,
Lane_Separation_Bar, PE_Drum, PE_Firewall_Orange/White, Steel_Barricade,
Concrete_Barricade01/02, Bollard01/04, concrete-barrier,
Bicycle_city/mountain, Scooter_01/02. 후자의 무인/실체 상태는 capture로 확인한다.
검증 안 된 신규 유모차/탑승자 자산을 이 pool에 이름만 보고 추가하지 않는다.

초기 asset group: 긴 barrier45% / box·drum·barrel35% / bollard10% /
검증된 무인 bicycle·scooter10%. 군 안은 DataID 균형, group/asset별 accepted GT를 감사한다.

방향 독립 quota: front1,200 / left600 / right600 / rear600. 후방을 줄여도 좌우를
같이 줄이지 않는다. 최종 Ego 방위각으로 front[-45,45),left[45,135),
rear[135,180]∪[-180,-135),right[-135,-45)를 정의하고 전/후측방 세부각도도 기록한다.

- front1,200: center0–10m480 /10–30m420 /30–60m240 /60–100m60.
  마지막60은 큰 장애물 우선이며 작은 박스를100m에 억지로positive 처리하지 않는다.
- left/right 각각600:0–5m300 /5–15m210 /15–30m90.
- rear600:0–10m360 /10–30m210 /30–60m30.
- 측방0–5m 중 표면 clearance0.3–2m 장면을 확보한다. center0.5m를 요구하는 것과
  다르다. Ego/객체 geometric footprint 비충돌 및 가시성을 먼저 확인한다.

미션1,200장: 접근/조기회피360 / 회피중 좌우 partial-view480 / 통과직후240 /
가림·여러장애물 복합120. 양쪽 회피를 대칭으로 만들고 긴barrier의 얇은 측면을 보호한다.
범용1,800장은 road-general 중심, 미션1,200장은 배포 회피 시나리오의 배치 구조와
planning_validator의 valid mission 구간을 참고하되 위치·거리·yaw·asset 조합을 변경한다.
배포 시나리오 원본은 수정하지 않는다.

긴 barrier yaw는 도로상대0/90°근처50%, 나머지[0,180)50%; box·barrel 등은[0,360)
균형 deck. 동적 경로 차량에 이 규칙을 적용하지 않는다.

## 6. 사람·동적/복합 관련2,500장

| 주 대상 | 범용 | 미션 | 합계 |
|---|---:|---:|---:|
| 일반 실제 사람(아이 포함) | 1,500 | 375 | 1,875 |
| NCAP dummy | 100 | 25 | 125 |
| 동물 | 100 | 25 | 125 |
| 반려동물 동반 사람 | 100 | 25 | 125 |
| 유모차 계열 | 100 | 25 | 125 |
| 자전거·스쿠터·탑승자 계열 | 100 | 25 | 125 |
| 합계 | 2,000 | 500 | 2,500 |

일반사람은 Man/Woman,police,worker,Boy/Girl의 검증된24종을 균형 sampling한다.
NCAP4종은 별도다. Dog1/2/Deer1과 복합 review16종 전체는 inventory에 보존하고
지원하는 동작과 annotation을 확인한 자산만 각 quota에 배정한다. 지원 안 되는 동작은
가짜 velocity 필드로 꾸미지 않는다. 보류종이 있으면 family를 임의로 없애지 않고
같은 family의 검증종 또는 사용자 합의한 대체 quota를 기록한다.

일반사람1,875: standing375 / walking938 / running 또는 검증된 다른동작562.
동물·복합은 가능한 경우 정지25% / 실제 이동·동작75%가 초기목표다.
로봇/무인 dummy에는 자연보행 animation을 있다고 가정하지 않는다.
각asset의 지원동작, 활성조건, animation phase는 live gate로 확정한다.
모든 동작을 같은시점에서 찍지 않고 안정 활성 후phase deck을 사용한다.

미션500: 횡단보도300 / 진로로 진입하거나 도로 가장자리에서 접근125 /
근접·부분가림·통과75. 범용은 서로 다른 도로/보도/횡단 위치에 방향·자세를 넓게 분산한다.
보행방향 좌→우/우→좌를 균형으로, 이동하는차량·자전거는 유효경로 위에 배치한다.
동적 장애물 미션의 자동차 등장은 차량 family 및 mixed scene 보조actor로도 coverage한다.

거리0–5m25% /5–15m40% /15–30m25% /30–50m10%.
방위front50% /left20% /right20% /rear10%. 각거리bin의 유효가시성/크기를 감사한다.
아기GT는Pedestrian, 개/유모차는Obstacle 후보, 자전거Vehicle의 CAR runtime 의미는
검토대기다. 원본component는모두보존한다. 미결매핑이있는scene은학습승인하지않는다.
동물Obstacle의 UNKNOWN 속도0 정책은 동적안전대응이아니며 별도tracker/runtime 검토가필요하다.

## 7. 다중 객체·가림·빈 배경

9,500 nonempty primary scene 중 single70%=6,650 / multiple30%=2,850.
multiple은2개70%=1,995 /3–4개30%=855장. 이는 **배치한 부모actor 수**이고
복합asset의component3개를다중actor3개로세지않는다.
거리·primary방향quota는기준target으로판정하고 보조GT는별도집계한다.
차량family 보조는 경로차량/정적장애물, 장애물family 보조는 다른장애물/정적차량,
pedfamily 보조는다른사람/차량/검증복합을사용한다. GT 개수폭증으로loss가한class에
몰리는지검사하고 필요시sampler/loss를별도로조정한다.

multiple 중30%(855장)는 영상상 부분가림을 목표로 한다. 물리적인mesh 관통은 금지한다.
전체가림GT를 무조건positive로 만들지 않는다. adjacent thin barriers, 앞뒤 두차,
사람옆자전거 등의 실제분리대상을 중복NMS로하나로합치지않는지도평가한다.
500 empty는고주로200/일반도로150/로터리100/횡단·보도50.
Tree/표지판/가로등은기존도로변맥락으로보존하고차로중앙랜덤positive로만들지않는다.
Fire/speedbump/manhole은현재학습scope제외. 배경scene에 미라벨동물/사람을의도적으로넣지않는다.

## 8. Ego 랜덤화와 상대객체의 불변조건

- Ego yaw:도로접선대비 ±3°60%, |offset|3–10°30%,10–15°10%; 부호대칭.
  이는초기안이며실제회피bag yaw분포로추후확인한다. 임의180°역주행은금지.
- 일반직선Ego 횡offset:0m40%,±0.15m30%,±0.30m30%.
  좁은합류/병목/로터리는0m50%,±0.10m30%,±0.15m20%로제한한다.
  차체모서리까지차로/주행가능영역에있는지검사하고큰yaw+offset불가능조합은재배치한다.
- Ego·정적객체yaw는독립seed. 실제주행NPC yaw/횡위치는그자신의MGeo경로를유지한다.
  상대차량의앞뒤거리·차선·차종은변화가능하나횡jitter/yawjitter는주지않는다.
- Actor yaw는도로기준,방향quota는최종Ego기준; 둘다metadata에기록한다.
  XY거리, longitudinal/lateral, geometric center와surface clearance도구분한다.
- Ego 초기속도0. 동작외관캡처는Ego정지+실제동적actor로우선구현한다.
  Ego주행이반드시필요한일부진단은독립profile로분리한다. GT planner/controller는
  주수집필수가아니고나중에closed-loop 보강/검증용이다.
- 날씨/시간/센서노이즈는이번10K에서추가무작위화하지않는다. 우선shape/view/range축을
  검증하고 추가domain randomization은별도실험으로분리한다.

## 9. 품질·split·실행·승인 gate

1. scene별4-camera/LiDAR/Instance/metadata를보존하고원본catalog/network/map hash와
   generatorversion,seed,primaryfamily/missiontag,actor/component,요청/실제bin을기록한다.
2. 지면/충돌/드리프트/실제movement/파일완결/GTgeometry/component중복UID를검사한다.
   동작scene의동작관측은Object 속도만아니라RGB/Instance 또는animation증거로확인한다.
3. object Topic은상태확인용이며GT는captureannotation이다. 복합uid→list parser가선행이다.
4. 먼거리<5points를모두자동drop하면camera-visible학습을잃을수있다. raw보존후camera
   visibility/minimumimage size/occlusion과point count를교차검토하는mask를구현·검증한다.
   아직이gate가없으므로희박원거리GT를무조건positive로승인하지않는다.
5. 장면위치+배치/animation variant의parentgroup을train/val85/15로분리한다.
   현재50mcell을그대로독립split로간주하지않는다:100m view는이웃cell을공유하므로
   서로다른map 구간과경계buffer를설계하고공유target/근접view를감사한다.
   작은맵이라완전공간분리가불가능하면한계를기록하고독립rosbag를최종test로쓴다.
6. v2 implementation 전componentparser/동작지원/100m학습ROI는미완료다.
   모델forward/back ROI는[-100,+100]목표이나LiDARradial100m경계를넘는
   diagonalcorner return을가정하지않는다. 횡BEV범위/voxel/image해상도는
   5060Ti smoke와4090목표latency를검토후확정하고기존±30m모델을몰래변경하지않는다.
7. 첫검증48scene:각majorfamily·모든yaw/방위·근접/원거리·동작·복합·가림을cover.
   별도exactquota계산/재현/범위/비충돌테스트후batch64이하설치·회수·resume한다.
   실패bin은같은quota로보충한다. 통신실패면중단,파일만생기면성공이라고보지않는다.
8. 10,000개scenario를미리MR폴더에쏟아넣지않는다. 생성/설치/정리목록과sidecar를
   ledger로관리하며사용자scenario는보존한다. 품질/속도pilot후본수집을승인한다.
9. Base3는3capture/scene였지만Base5는1capture/scene이라load횟수가약3배다.
   이전10K시간을그대로약속하지않는다. scene load+동작wait+capture+retry실측으로
   예상시간/용량/진행률을산출한다. 긴수집에accepted-by-bin monitor가필수다.
10. 평가에는새/기존mission rosbag의detection/tracking/Map-CV를함께쓴다.
    원거리first detection/planning전달거리,근접ID/box안정성,보행동작recall을포함한다.
    offlineAP만으로E25를교체하거나trackervelocity개선까지달성했다고주장하지않는다.

## 10. 다음 구현 순서

policy v2 exact deck →component-preserving converter/class매핑 →동작지원probe →
Ego pose/footprint·final Ego bin audit →48scene smoke →range/visibility gate →
학습ROI/dataloader GPU smoke →accepted quota resume runner →본10K수집.

이번commit은설계문서만추가한다. 기존v1실행config는과거pilot재현용으로유지하며
위설계로수집할때는새versionedconfig를명시적으로만든다. 대량생성/로드/학습은하지않았다.
