# Base5 정적 구역 확장과 크기 변형 검증

문서 상태: EVIDENCE. 기준일: 2026-10-16, Asia/Seoul.
새 본수집과 학습은 실행하지 않았다. 기존 Sunny13 5K·E24·v154 release와 실패 원본은 보존한다.
현재 운영 판정은 [Base5 CURRENT](BASE5_V2_CURRENT.md), 설계 의도는 [확장 계획](261016_base5_mission_extent_and_asset_scale_plan.md)을 따른다.

## 정적 미션의 공간 분포

새 `candidate_v155b`는 주요 객체 중심을 전체 MGeo 경로에 투영해 전·안·후를 구분한다.
Ego 위치만 통과 후라는 이유로 객체의 통과 후 사례로 세지 않는다.

| 주요 객체 위치 | v154 | 새 CPU 후보 | 실제 경로 진행거리 |
|---|---:|---:|---|
| 삼거리 전 | 440 | 300 | 251.16–316.09m |
| 삼거리 안 | 160 | 120 | 316.80–346.76m |
| 삼거리 후 | 0 | 180 | 347.30–390.53m |

600장만 다시 생성하고 나머지 4,400개 entry와 payload를 그대로 보존했다.
주요 자산 122종, train 444·val 156, phase·yaw·gap·seed·actor identity를 유지했다.
초기 240/180/180안은 안쪽 link가 val인 기존 split과 충돌하므로 변경했다.
통과 후는 train 144·val 36이다. 신호등 인지와 회피의 상호작용은 배치 제약에 추가하지 않았다.

2m 구간의 사용 칸은 전 34·안 16·후 23개이고, 한 칸의 최대 장면 수는 16·13·19장이다.
600개 primary XY는 고유하다. 전체 5K Ego XY는 mm 단위로 4,998곳이며,
같은 Ego 위치를 쓴 두 쌍은 서로 다른 객체 구성이다. 5,000개 payload는 모두 고유하다.

CPU 후보는 `artifacts/evaluation/base5_static_extent_261016/candidate_v155b/manifest.json`이다.
SHA-256은 `b1f69b455e795ee85a46aeac30c28751b9d865aa6abb0649fa8f36fac1968098`이다.
전체 시나리오 감사는 `OFFLINE_PASS`, 보존 감사 오류는 0개다.

전·안·후와 split·4개 phase를 교차한 16장에 9개 정적 객체 pool을 포함해 native 시험했다.
첫 결과는 14장 승인·2장 review였다. Dog2의 43.88m 접근과 concrete-barrier의 1.78m 측방 장면은
`primary_no_sensor_evidence`였고, 충돌·Ego 정지·종료 Park 실패는 없었다.
이 둘을 접지 실패라고 해석하거나 정답을 강제로 추가하지 않는다.
Dog2는 원본 영상에 약 3.41×9.50픽셀·mask 27픽셀로 남았지만 당시 320 입력의 최소 가시성 조건을
넘지 못했다. 완전히 보이지 않았다는 뜻이 아니다. barrier는 실제 근접 사각 사례였다.
동일 자산·phase·구간·split·크기 계약을 보존하는 유한 재생성에서 barrier는 2.38m의 새 배치로
승인됐다. Dog2의 첫 두 재생성은 62.47m·51.78m에서도 review여서 그대로 보존했다.

## 확대된 물체와 원형 크기의 요약 정답

CargoBox를 같은 위치에서 축별로 확대하자 화면·2D 박스는 커지는데
`LIDAR_5/*_Instance.txt`의 size는 원형 `[0.7486297, 0.7486296, 0.4344158]`에 고정됐다.
반면 동일 timestamp의 `*_Instance_8Points.txt`는 이미 확대된 8개 모서리를 저장했다.
모서리 평균과 원래 중심은 약 0.01mm 수준으로 맞고, 회전도 원래 annotation과 일치했다.

CargoBox의 실측 축 대응은 native L/W/H ← Unity Z/X/Y다.
예를 들어 Unity `(2,3,2)`의 native 치수는 대략 `(1.4973, 1.4973, 1.3032)m`다.
한 자산의 결과만으로 다른 자산의 형상·접지·자연스러움을 승인하지 않는다.

6m·yaw 0/90도 비교에서 원형은 각 26점, 등방 1.5배는 82/83점,
샘플 `(2,3,2)`는 각 332점이 실제 박스 안에 있었다. 원형 size를 그대로 사용하면
등방·샘플 조건의 점 수가 0점으로 계산됐다. 샘플의 심각한 부유 판정도 실제 8모서리로 계산하면
지면 간격 약 2.8–4.2cm로 바뀌었다. 실제 부유와 잘못된 박스 크기로 인한 판정을 구분해야 한다.

## 원본 모서리를 이용한 정답 경로

크기 변형을 명시적으로 등록한 허용 Obstacle 부모에만 다음 경로를 적용한다.

1. 저장된 scenario의 DataID·UID·scale·정적 동작을 metadata와 대조한다.
2. 정확히 같은 파일 stem의 Instance와 8Points를 읽고 원문 해시를 고정한다.
3. 부모·클래스·component 수, 중심·회전, 대칭과 8개 꼭짓점 조합을 검증한다.
4. 원래 RPY 좌표계에서 모서리의 범위를 계산해 실제 size를 얻는다. scale을 다시 곱하지 않는다.
5. 같은 박스를 점 개수·지면 평면 제외·접지 간격·Ego 간격·upright GT 변환에 사용한다.

원본 파일·중심·회전은 변경하지 않는다. 파생 component에 원형 요약 size와
`native_corner_geometry`의 파일 해시·잔차·정책을 함께 남긴다.
scale 1, 차량, 보행자·동물·복합체에는 이 경로를 자동 적용하지 않는다.
입력 검증기는 파생 크기를 원본 8모서리에서 다시 계산해 대조하며 위조·누락·원문 변경을 거부한다.

## 자산별 규격과 배치 초기화

첫 대표 시험은 8종 × 원형/두 확대 규격 × yaw 0/90도의 48장이었다.
32장 승인·16장 review였고 종료 Park를 확인했다.
RedBarrel·PE_Firewall_Orange·Steel_Barricade·Concrete_Barricade01은
기존 배치용 형상 참조가 현재 원형 native 치수보다 컸다. 이로 인해 배치 예상치와 실측 크기를
대조하는 새 검사가 불일치를 검출했다. 원본 정답이나 검사 기준을 바꿔 통과시키지 않고,
두 방향의 원형 관측으로 배치용 unit 참조를 분리한 뒤 다시 시험했다.
이 문제는 배치 예상치의 차이이며 기존 scale 1 학습 GT가 잘못됐다는 뜻이 아니다.

수정된 초기 배치로 4종 × 원형/두 규격 × yaw 0/90도 24장을 새로 촬영해 **24/24 승인**됐다.
native 8모서리와 audit 크기가 같고, 모서리 투영의 instance mask 포함률은 최소 99.95%였다.
측정된 지면 간격은 약 −1.3mm~+7.3mm이며 QA 임계값은 완화하지 않았다.
앞선 48장 중 승인된 32장과 재시험 24장은 별도 cohort로 Q90·upright·3클래스·512 입력의
실제 CPU 변환·점 수 재계산·원본 불변 검사를 통과했다. 학습이나 AP 시험은 아니다.

| 첫 승인 자산 | 이산 규격 | 영상 판단 |
|---|---|---|
| CargoBox·WoodBox | 등방 1.25/1.5배 | 다른 규격의 운반 상자로 자연스러움 |
| RedBarrel·YellowBarrel·PE_Drum | 등방 1.25/1.5배 | 큰 산업용 통의 범위로 판단, 큰 규격은 낮은 비중 |
| PE_Firewall_Orange·Steel_Barricade·Concrete_Barricade01 | Unity X 길이 1.25/1.5배 | 높이·두께는 유지하고 길이만 다른 방호물 |

같은 영상 창의 원형·두 규격을 직접 비교해 채택했다. 허용 후보 12종 중 나머지 4종은
이번 native·외형 승인이 없으므로 원형만 유지한다. CargoBox의 공식 `(2,3,2)`도 형상·모서리는
확인했지만 이번 생성기의 승인 규격에는 넣지 않았다. 검토 후보와 실제 채택을 구분한다.

배치의 pivot·MGeo 높이는 초기화 근사값이다. 절대 `/Object_topic` Z와 비동기 Ego pose를
조합한 값은 native 센서 좌표계의 접촉 정답으로 사용하지 않는다. 기존 저장 높이와 지도 초기화,
동일 장면의 native 모서리·영상·LiDAR 지면 간격을 함께 사용한다.
실제 bbox size는 native 8모서리에서 얻고, 배치용 offset 근사와 구분한다.
자산별 unit 참조는 `unit_geometry_reference_v155e2.json`, 이 단계의 8종 근거 정책은
`qualified_scale_8assets_v155.json`이다. 둘 다 아래 실험 artifact 경로에 있으며 원형·변형의
원본 파일과 visual review의 해시를 포함한다.

크기 변형은 별도 family가 아니다. 허용 자산의 기존 actor slot에 이산 규격을 동결한다.
box 30%, barrier·drum 20%를 시작 비중으로 하되 원형·희귀 자산·primary를 보호한다.
동결 전에만 크기 budget과 기존 거리 tier를 검사해 불가능한 규격을 명시적으로 제외한다.
동결 후 실패했다고 scale 1로 되돌리거나 다른 actor를 몰래 삭제하지 않는다.

## 새 5K의 통합 생성 결과

`candidate_scale_v155g/manifest.json`은 정적 구역 확장·유한 가시성 재배치 위에 크기 정책을 통합한
새 5,000개 시나리오다. 실제 5K 캡처 완료를 뜻하지 않는다.
전체 CPU 감사는 `OFFLINE_PASS`, source/runtime 보존 오류는 0개다.
크기 변형은 **338장·405개 부모**에 들어가고 나머지 4,662개 entry/payload는 직전 source와 같다.
자산·actor 수·split·거리/방향 계약은 유지했다. 6개 WoodBox는 동결 전 크기 budget 검토에서
1.5배 대신 이미 검증된 1.25배로 정했으며, 원형 강등이나 actor 삭제는 없었다.

| 자산 | 변형 부모 수 | 허용 부모 중 목표 비중 |
|---|---:|---:|
| CargoBox | 68 | 30% |
| WoodBox | 74 | 30% |
| RedBarrel | 44 | 20% |
| YellowBarrel | 45 | 20% |
| PE_Drum | 40 | 20% |
| PE_Firewall_Orange | 44 | 20% |
| Steel_Barricade | 47 | 20% |
| Concrete_Barricade01 | 43 | 20% |
| 합계 | 405 | 자산·split별 반올림 배분 |

5K SHA-256은 `e09e0d95436a6b1188bdde1e817d10b20f21decf470e2975202e6955a4ff50ca`다.
이 snapshot의 `bulk_ready=false`는 생성만 마쳤다는 뜻이다. 이후 native smoke·변환·재개 검증의
별도 receipt를 확인하며 과거 snapshot의 상태를 사후 수정하지 않는다.

정책 근거를 actor마다 반복 검증하던 CPU 병목도 제거했다. 시작 시 전체 근거를 검증해 깊은 불변
정책을 만들고 종료 시 원문 해시·파일 상태를 다시 대조한다. 실제 약 807KB 정책의 actor 선택은
48.52ms→0.061ms였고 결과는 같았다. 검사를 없애거나 변경 가능한 dict를 신뢰하는 캐시가 아니다.

## 병행 반영된 정식 센서 설정과 재동결

작업 중 별도 커밋 `fc2e6017`이 현장 export의 float32 표기로 센서 주기를 통일했다.
GPS 1개·Camera 4개의 `sensorPeriod`가 `0.03333333333333333`에서
`0.03333333507180214`로 바뀌었고 다른 JSON 값은 같았다. 센서 위치·해상도·LiDAR 주기나
캘리브레이션 기하 변경이 아니며 완료 데이터·모델을 폐기할 이유가 아니다.

그러나 파일 SHA는 `35615d2a…ee45dd`에서 `35c4ae55…f82c2`로 달라져 v155g의 혼합 smoke
등록이 안전하게 중단됐다. 이는 단위 테스트의 센서 오염이 아니라 병행 반영된 정식 변경이다.
기존 센서로 되돌리거나 hash 예외를 넣지 않고, 기존 CLI로 far·지면 준비·registry·새 source를
새 경로에서 다시 동결한다. v155g와 앞선 캡처·실패·정책 근거는 변경하지 않는다.

현재 센서에서 exact 두 음영 장면의 지면/정적 기준과 실제 이동을 다시 검사해 native **2/2 승인**을
확인했다. Dog2 정적 기준은 `actor_ground_plane_unverified`여서 보존하고, 기존의 제한된 근접
지면 관측 1회를 거쳐 경로를 준비했다. 준비 캡처 5회와 승인 이동 2회이며 모두 종료 Park를 확인했다.
원본은 `artifacts/evaluation/base5_sensor_refresh_261016/shadow2_ground_current_v155j/`다.
첫 인덱스 오선택 진단 `_v155i`는 별도 보존하고 shadow registry·판정·학습 데이터에 사용하지 않는다.

정상 CLI로 재준비한 far/spread full은 이전 v154와 **전체 5,000개 scenario가 동일**하고 far 160개의
실제 파일도 동일했다. 센서 pin과 파생 metadata만 새로 동결했다. 이후 정적 구역·크기 정책을
다시 통합한 `candidate_scale_v155j`도 변형 338장/405개와 16개 규격별 장수를 유지했다.
v155g와 다른 실제 배치는 초기 가시성 재생성 두 장면뿐이다. 이 새 source는 과거 recovery의
성공·실패를 지우지 않으며, 센서 재동결이 배치 난수를 바꿨다고 해석하지 않는다.

이 단계의 전체 source는 `artifacts/evaluation/base5_mission_scale_261016/registered_scale_v155k/manifest.json`,
SHA-256 `d97ef324df924d0130bc7c3ad92a464ef352c63356485abd185e741251de585b`다.
그 위에 새 음영 준비를 exact 두 장면만 등록했다. 실제 raw를 학습 데이터로 재사용한 것이 아니다.
정확히 같은 source에서 미리 고른 17장 smoke는 16개 크기 규격·변형 primary/보조·범용/정적 미션/음영/
다중 객체와 기존 이동·원거리 보호 8장을 포함한다. 정적 구역·phase·split의 개별 범주를 덮지만,
이 17장만으로 구역×phase 모든 교차칸을 다시 검증했다고 주장하지 않는다.

## 보충 경로와 전달 경계

확대 장면의 보충은 seed·자산 목록·UID·동작·scale 규격·split·거리 계약을 고정한 채
새 geometry만 유한 탐색한다. 원형 donor로 바꾸거나 보조 객체를 빼지 않는다.
512회 proposal·8개 유효 후보·저장된 세대 ordinal 상한을 유지하며 상한을 재실행으로 초기화하지 않는다.
실제 실패하지 않은 원본을 실패 ledger로 꾸미지 않고, 별도 counterfactual probe 두 장을 준비했다.

PE_Drum 1.5배 primary의 2객체 장면과 YellowBarrel 1.25배 보조의 7객체 장면을 새 배치로
native 촬영해 **2/2 승인**했다. 이는 보충 geometry 생성·native GT·간격·안전 경로의 실증이며
실제 production 실패 뒤 자동 보충이 끝났다는 주장은 아니다.
첫 한 장 뒤 안전 중단·재개에서 기존 raw 37개와 ledger/event 앞부분이 그대로였다.
완료 재개는 ROS import와 socket 생성을 차단한 상태에서 실행해 **추가 촬영 0회**, 보호 파일 123개
불변을 확인했다. 원본은 `completion_probe_v155k`·`capture_completion_probe_v155k`,
receipt는 `completion_probe_v155k_partial_resume.json`·`completion_probe_v155k_terminal_resume/report.json`이다.

raw scenario의 nonunit UID와 명시적 metadata가 정확히 같아야 한다. metadata의 scale·native 모서리
근거를 빼거나 원형 요약 크기로 되돌린 audit는 재검사·완료 resume·공통 GT 변환에서 거부한다.
portable 전달은 원본 audit를 수정하지 않고 명시한 source root만 옮긴다. 상대 센서 경로·timestamp
basename·원문 SHA/bytes·UID·모서리 기하는 그대로 대조하며, 일반 hash 검사 옵션 OFF도 이 검사를 끄지 않는다.

첫 혼합 smoke의 보호 8장은 통과했지만, 확대 장면은 촬영 전 catalog 검사에서 중단됐다.
원인은 lean runtime metadata를 자산 catalog처럼 읽어 semantic 필드가 빠졌던 것이다.
실제 catalog를 읽어 검증하도록 고치고 모든 5,000장에 collector의 실제 preflight를 실행해 통과했다.
검사 기준을 약화하지 않았으며 잘못된 크기의 GT를 승인한 사건이 아니다. 중단 세션은 그대로 보존하고
현재 runtime으로 새 registration·새 native 세션을 사용한다.

## 혼합 장면의 추가 관측과 최종 7종 정책

catalog 검사를 수정한 `capture_smoke_v155l`은 최종 선택 17장이 승인됐지만, 최초 관측 두 장에
`scaled_native_bbox_size_mismatch`가 남았다. 유한 동일 셀 재생성에서 승인됐어도 이 실패를 지우거나
17/17 무실패 시험이라고 표현하지 않는다. 두 장 모두 RedBarrel이 문제였고 다른 변형 자산은 일치했다.

02170의 UID 8503·1.25배와 04192의 UID 8506·1.5배는 native 기본 치수가 이전 기준보다 정확히 약
1.25배였다. 원본 8모서리는 그 기본 치수와 요청 배율에 맞는 실제 물체를 기록했고, 같은 UID·DataID·
요청 배율의 재생성에서는 작은 기본 치수가 관측됐다. 원형 RedBarrel에도 기본 치수 차이가 있었다.
`Object_topic.size`는 이 차이를 반영하지 않아 원인이나 정답 대체 근거로 쓰지 않는다.
캐시·LOD·시뮬레이터 내부 동작 중 무엇이 원인인지는 입증하지 못했다.
독립 대조표 `redbarrel_native_variation_v155l.json`은 4회 캡처·12개 RedBarrel의 원문 24개를
대조한다. 모서리 재구성의 최대 잔차는 약 0.034mm였고 원본 변경은 없었다.

따라서 QA 허용 오차를 넓히거나 GT를 억지로 줄이지 않고 **RedBarrel의 확대 두 규격만 보류**했다.
원형 RedBarrel 221개와 자산 coverage는 유지한다. 상자 2종·통 2종·방호물 3종의 **7종·14규격**은
기존 native·영상 근거와 배율 비중을 그대로 사용한다. 별도 family나 추가 5K를 만드는 방식이 아니다.
`qualified_scale_7assets_v155m.json`과 `deferred_redbarrel_v155m.json`에 원본 실패·정책·해시를 연결했다.
기존 8종 정책과 실패 원본은 역사적 근거로 보존하며 사후 변경하지 않는다.

새 `candidate_scale_v155m`의 변형은 **315장·361개 부모**다. 앞선 405개에서 RedBarrel 44개만 제외했다.
나머지 자산별 부모 수는 CargoBox 68·WoodBox 74·YellowBarrel 45·PE_Drum 40·
PE_Firewall_Orange 44·Steel_Barricade 47·Concrete_Barricade01 43개다.
actor 종류·수·identity, seed·family·mission·split은 동일하다. 전체 CPU 감사와 보존 감사를 통과했다.

최종 혼합 시험도 결과를 보기 전에 정확히 17장으로 등록한다. 승인 14규격과 확대 primary/보조·
범용/정적 미션/음영/다중 객체, 전·안·후·4개 phase·양 split 및 기존 이동·원거리 보호 8장을 포함한다.
교차된 모든 범주를 이 17장만으로 전수 검증했다고 주장하지 않는다.

### 최종 후보의 실제 캡처와 학습 입력

`registered_scale_v155m/manifest.json`의 SHA-256은
`962854897cca976290169564549c173542fe78f624f5bf1d566706005b1ffff4`다.
`qualified_smoke_v155m`은 runtime 95개·입력 13,628개의 해시를 동결한 뒤 촬영했다.
`capture_smoke_v155m`은 **17/17 승인·최초 review/failed 0건**이며, 실제 8개 하위 세션의
종료 Park와 각 원본의 CollisionData 수신·무접촉 근거를 확인했다. 실제 경과는 약 136초지만
이 의도적 검증 표본으로 전체 5K 시간을 외삽하지 않는다.

첫 4장 뒤 안전 종료·재개에서 앞선 raw와 승인 목록이 불변이었다. 완료 후 동일 명령의 resume도
실행해 **추가 촬영 0회·raw/selected/누적시간 불변**을 확인했다.
부분 재개 receipt는 `smoke_v155m_partial_before.json`·`smoke_v155m_partial_resume.json`,
완료 재개 receipt는 `smoke_v155m_terminal_resume.json`이다.

실제 selected ledger 하나로 Q90 JPEG·upright·3클래스·512×1056 CPU 입력을 만들어 검사했다.
native component 44개 중 기존 가시성 정책에 맞는 **43개가 유지**됐고, 예상 유지 component의
누락·추가 오류는 모두 0이었다. 제외된 1개는 센서 지원 기준 미달 Lane_Separation_Bar이며,
확대 객체를 잘못 버리거나 클래스·parent/component를 바꾼 결과가 아니다.
승인 확대 14규격은 각각 실제 GT 1개씩 모두 유지됐다. 이 중 LiDAR 0점인 두 방호물도
512 카메라 근거로 유지했다. 원거리 보호 대상에는 camera-only 차량 1개가 포함됐다.
profile별 LiDAR 0/1–4/5–19/20점 이상은 2/4/5/3개이며 카메라 조건은 14/14 충족했다.
`profile_training_support_v155m.json`에서 실제 three-class infos의 identity·upright bbox까지 대조했다.
차선 분리봉은 32.49m에서 mask 17픽셀이 남았지만 너비 3.526px로 최소 4px에 못 미쳤고 LiDAR도
0점이었다. 완전히 안 보였다는 뜻은 아니며 기존 가시성 정책의 결과다.
모델 학습·검출 AP·실전 추론 성능을 검증한 것은 아니다.

위 결과의 원문은 `training_input_qualification_v155m/report.json`이다.
최종 후보의 전방 방호물 혼합·통과 상자 혼합·음영 WoodBox의 원본 RGB도 직접 대조했다.
원형 대비 확대 여부의 첫 외형 판단은 앞서 보존한 자산별 대조 영상 근거와 함께 해석한다.

## 재현 경로와 준비 상태

실험 원본과 보고서는 `artifacts/evaluation/base5_mission_scale_261016/` 아래에 있다.
첫 축 시험과 근접 시험의 review·실패 기록은 새 판정으로 덮어쓰지 않았다.
최종 `release_v155m/release.json`은 현재 Sunny13의 **`bulk_ready=true`·blocker 0개**다.
release SHA-256은 `99428ed55d8c538d2eb056c66152ebd78cd3d59e9c2d18130b009cbe1fc12629`다.
전체 5,000개에 실제 collector 입장 검사를 수행했고 source는 수정하지 않았다.
JUnit을 중복 제거한 CPU 회귀 1,545개가 통과했다. host Torch가 필요한 과거 Base3 segmentation
검사 1개는 실행하지 않았으며, 대시보드 headless 검사는 별도 실제 실행해 통과했다.
확대 장면 실패 뒤 자동 보충까지 별도 실증한 것은 아니므로 `completion_live_qualified=false`다.
기존 생성·유한 재배치와 counterfactual 2장 근거를 production 실패 처리 완료로 부풀리지 않는다.

다음 명령은 **본수집을 실행하지 않는** 현재 release·코드·입력 해시 검사와 준비 명령이다.

```bash
python3 dataset/morai_od/scripts/launch_base5_obstacle_support.py \
  --release artifacts/evaluation/base5_mission_scale_261016/release_v155m/release.json \
  --environment sunny13
```

실제 본수집을 시작할 때만 MR UI의 Sunny13·수집 센서·네트워크와 통제권을 확인한 뒤
위 명령에 `--run --confirmed-environment sunny13`을 추가한다. 이 작업에서는 실행하지 않았다.
`--complete`는 이번 release에서 거부된다. 본수집 후 부족분을 보존하고 새 확대 계약의 보충을
별도 검증한 뒤 진행한다. 기존 release나 scale 1 donor로 바꿔 이 보호를 우회하지 않는다.
운영 예산은 보수적으로 8–12시간·45GiB이며 새 5K 전체의 실측 시간은 아직 없다.
다른 다섯 환경은 최신 정책의 새 배치와 별도 native smoke가 필요하다.
v154 release와 초기 8종 정책은 당시 이력이지 현재 코드의 대체 승인이 아니다.
이 작업은 5K 본수집이나 학습 AP 향상의 인증이 아니다.

주요 구현은 `base5_asset_scale.py`, `base5_native_box_corners.py`,
`base5_scale_capture_contract.py`, `base5_static_mission_extent.py`다.
준비 CLI는 `prepare_base5_static_mission_extent.py`, `prepare_base5_scale_probe.py`,
`prepare_base5_native_scale_policy.py`, `prepare_base5_asset_scale.py`다.
