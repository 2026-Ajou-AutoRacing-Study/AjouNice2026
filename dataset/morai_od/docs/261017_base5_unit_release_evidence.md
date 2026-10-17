# Base5 원형 수집 최종 구성과 재개 검증

문서 상태: EVIDENCE. 기준일: 2026-10-17, Asia/Seoul.
확대 수집은 중단한다. 기존 완료 Sunny13 5K·E24·native GT는 보존하고, 다음 원형 5K의
최종 구성·지면 준비·부족분 회복·학습 입력을 검증한다. 5K 본수집과 학습은 실행하지 않는다.

현재 승인본은 아래의 **v158a**다. 최종 exact 32/32·CPU 회귀 2,041개·실제 학습 입력
전처리·부분/완료 재개를 통과해 `bulk_ready=true`로 동결했다. 이는 Sunny13의 수집 준비
승인이며 AP 향상이나 30K 전체의 승인이 아니다. v157a와 실패한 v3는 원인·변경 이력이다.

## v157a 원형 기준 구성

artifact root는 `artifacts/evaluation/base5_unit_release_261017/`다.
`candidate_v157a/manifest.json`의 SHA는
`c6a5910df5d35a1f974685a8ebf677be36bc9a13e8fee4b9db91637be0f9bc9c`다.
확대 이전 원형 source에서 등록된 음영 지면 준비 두 장만 먼저 적용하고, 물리 19종의 원형
치수 참조와 지도 전역 학습 분할 overlay를 순서대로 붙였다. 원래 payload 4,998장과
모든 frozen spec·자산 할당·배치 수·동작 계약을 보존했다.

| 항목 | 실제 구성표 감사 결과 |
|---|---:|
| 전체 장면과 고유 payload | 5,000 / 5,000 |
| 원래 capture split | train 4,255 / val 745 |
| 파생 학습 split | train 4,250 / val 750 |
| 관측된 Ego link 중 train 포함 | 69 / 69 |
| 관측된 공간 셀 중 train 포함 | 316 / 316 |
| train에 포함된 자산 | 122종, 각각 최소 5개 배치 그룹 |
| 물리 장애물 부모 0~20m | 2,797 |
| 물리 장애물 부모 20~30m | 865 |
| 물리 장애물 부모 30m 이상 | 282 |
| 차량 부모 50~80m | 전방 269 / 후방 210 |
| 차량 부모 80~100m | 전방 168 / 후방 153 |

객체 수와 거리는 **계획한 부모 배치**의 분모다. 실제 가시 component GT·LiDAR 지원·AP가 아니다.
범용/미션 비율·정적 구역 전/중/후 300/120/180·근거리 장애물 보강·원거리 차량 보강은 유지한다.
objectList 4,328개는 숫자형 scale과 존재하는 UI mirror가 모두 111이다. 다른 컨테이너의
기본 serializer가 scale을 생략한 경우는 별도로 세며, 숫자 111이 명시됐다고 쓰지 않는다.

조립 명령·중간 source·보존 해시는 `build_receipt_v157a.json`, 전수 분포와 입력 pin은
`source_distribution_v157a.json`, 전체 scenario 검사는 `scenario_audit_v157a.json`에 있다.
기존 [원형 89장 검증](261017_base5_unit_only_restart_validation.md)은 별도 안정성 근거이며
이번 최종 smoke의 새 승인 수에 더하지 않는다.

## 실제 이동의 준비 상태

| 장면 상태 | 장면 수 | 수집 전 필요한 처리 |
|---|---:|---|
| 정적·정지 외형 | 4,822 | 기본 단일 캡처 QA |
| 사거리 editor 높이 경로 준비 | 154 | 등록된 두 waypoint·원문 경로와 높이 증거 확인 |
| 음영 독립 terrain 준비 재사용 | 2 | 등록된 정확한 입력만 재사용하고 새 native 취득 |
| 음영 natural 경로 준비 대기 | 22 | 장면별 독립 terrain·정지 외형 측정 후 실제 이동 취득 |

준비 대기 22장은 사람·동물·복합체의 walking/running 장면이며 정적으로 바꾸거나 제외하지 않는다.
전체 이동 178장이 이미 지면 준비됐다는 뜻이 아니다. 실제 source를 전수 분류한 inventory와
미준비 ID 목록을 사전등록 및 release에 보존한다.

## 지면 검사에서 상부 구조와 도로를 구분

이전 00351의 원형 장면은 약 58m 객체 주위의 희소한 점으로 평면을 맞추면서, 도로보다
약 6.68m 높은 표면을 지지면으로 해석했다. native 바닥은 도로 초기값보다 약 0.217m 높았다.
상부 구조 후보를 잘 맞춘 평면이라는 이유만으로 실제 6m 침하가 확정되는 것은 아니다.

새 분류는 정적·비보정·실제 단일 native 성분·로컬 범위 밖·희소 다중 높이·신뢰 가능한
도로 초기값과 실제 위치의 대조가 모두 있을 때만 `surface association unverified`로 남긴다.
실제 지면과 연결된 심각한 부유·침하, 동적 객체, 독립 reference 불일치·GT 누락·충돌은
기존 검사를 유지한다. 원문 plane·gap·native는 고치지 않으며 추가 캡처를 요구하지 않는다.
구분 임계값은 출제 확률이나 지면 접촉 정답이 아니라 제한적인 판단 불가 분류 조건이다.

기존 89장 재계산에서는 00351의 최초/반복 두 관측만 분류가 달라졌다. 다른 센서·가시성·
runtime 누락 사유는 그대로다. 기존 ledger의 review 7장은 소급 승인하지 않았다.
원본·ledger 3,315개 해시가 전후 동일했다. 별도 보고서는
`artifacts/evaluation/base5_unit_only_stability_261017/static_surface_association_reanalysis_v1/report.json`에 보존한다.

## 회복과 지면 준비의 계약 보강

새 QA·분할 태그를 처음 붙인 경로뿐 아니라, 이후 배치를 바꾸는 경로에도 이어야 한다.
내부 회복은 같은 primary·자산 구성·동작·거리·미션 계약을 유지하고 새 payload를 만든다.
새 후보는 원래 canonical 부모를 명시하고 별도의 overlay에 등록한다. 중첩 회복도 실패한
직전 별칭을 새 부모로 오인하지 않으며, 원래 source·overlay·raw를 덮어쓰지 않는다.
해시·원형 QA·계보 오류는 전체 중단 사유다. 유효한 후보가 배치 조건을 만족하지 못하는
경우만 기존 제한 안에서 탐색하고 부족분으로 남긴다.

`base5_calibration_layout.py`는 새 준비 결과를 별도의 파일로 정규화한다.

- terrain/정지 외형 측정은 `calibration_only`를 유지하며 학습 overlay와 quota에서 분리한다.
- 진단 actor는 원래 UID·자산·컨테이너의 부분집합만 허용하고 원형 QA·충돌·Park는 유지한다.
- 최종 학습 장면의 좌표·경로가 달라지면 canonical recovery API로 다시 등록한다.
- 같은 payload의 정책 metadata만 바꿀 때는 Ego·객체 순서/좌표·spec·quota를 바꿀 수 없다.
- 준비 결과 재사용은 입력·helper 해시와 필수 출력 및 전체 파일 목록이 모두 같아야 한다.

원래 준비 파일은 보존한다. 변경된 코드로 과거 output을 강제 resume하지 않는다.
등록된 두 음영 입력의 재사용과 나머지 22개의 새 terrain 준비를 혼동하지 않는다.
대표 3장면의 empty/static reference 6개 CPU 정규화·cache 검산은
`calibration_normalization_cpu_v2/summary.json`에 있으며 native 취득 승인은 아니다.

### 실제 음영 준비와 부족분 회복 대조

자연 음영 경로의 Woman7·Deer1·Ped_Pet1 세 장면을 별도 진단했다. 준비용 12회와 최종
이동 3회를 취득해 최종 3/3을 승인했고, 모두 충돌 검사와 종료 P를 확인했다. Ped_Pet1의
정지 참조 한 번은 검토로 보존한 뒤 새 배치로 준비했으므로 전체가 첫 시도에 성공한 것은 아니다.
이 시험은 최종 반복 방지 수정 전 호출 코드의 결과다. 보고서는
`terrain_diagnostic_independent_audit_v1.json`이며 최종 source의 미준비 22장을 모두 준비했다고
해석하지 않는다. 이 세 진단 결과를 full5K source에 소급 적용하지도 않았다.

실제 부족분 시험 v1에서는 최종 2/2을 채웠지만, 실패한 동일 payload를 내부/외부 차수에서
두 번 촬영한 문제가 드러났다. 최종 승인 수만으로 성공이라고 판정하지 않고 원문을 보존했다.
현재는 이전 차수의 실제 취득 SHA를 공유해 후보 생성 중 제외하고, 로드 직전에도 중복을
차단한다. 센서 저장 전 안전한 도착 실패의 기존 6→9초 재확인은 별도로 유지한다.

같은 대조 입력으로 새로 등록한 v2는 2/2을 61.0초에 채웠다. 실제 native 취득은 4회·고유
payload 4개·차수 간 중복 0이었다. 모든 세션의 충돌 검사·종료 P, 원형 물리 부모 19개의
8모서리·치수·정체성·계보를 검산했다. 현재 runtime 425개와 입력 pin 18,639개가 일치했고
취득 근거 221개 파일의 해시·inode·mtime·크기는 분석 전후 같았다. 이는 두 입력의 회복
대조이지 독립 장면 4개의 승인이나 별도 완료 보충기의 live 승인이 아니다. 독립 보고서는
`recovery_diagnostic_independent_audit_v2/report.json`이다.

## 학습 분할의 최종화

배치 교체나 지면 보정으로 실제 geometry가 바뀌면 원래 full5K overlay를 그대로 학습에
쓰지 않는다. 여러 cell의 독립 overlay는 각각 제안할 때의 부모 계약을 검증한 것이며,
선택한 전체 실제 배치끼리 새 중복·누수·train coverage 문제가 없는지는 다시 확인해야 한다.

완료 후 `finalize_base5_layout_split.py` 계열의 전역 최종화로 공통 overlay와 파생 선택 ledger를
만들고, 그것으로 JPEG·upright·학습 정보를 변환한다. 서로 다른 옛 overlay를 행별로 무조건
허용하는 fallback은 넣지 않았다. 원본 28 + recovery 2의 합성 fixture는 공통 overlay로 실제
30행 변환을 통과했고 옛 미통합 overlay는 거부됐다. 이는 실제 native 승인 분모가 아니다.
검산 근거는 `converter_overlay_review_v1/report.json`이다.

## v157a 사전등록 32장 결과와 차단

`registered_smoke_v3`의 32장은 source 결과를 보기 전에 metadata로 선정했다.
`capture_smoke_v3`는 362.6초에 31/32 source-cell 승인을 얻고 검토 1장으로 종료했다.
원본 exact payload 승인은 30장이다. `00217` 한 장은 정상적인 온디맨드 지면 준비 후
`00217_groundpath`로 승인됐으므로 원본 exact 관측과 같은 수로 세지 않는다.

| 분모 | 실제 결과 |
|---|---:|
| 최종 quota 관측 | 32회: 승인 31 / 검토 1 |
| 검토 장면의 새 geometry 재시도 | 3회, 모두 검토 |
| 지면 준비용 calibration | 3회, quota 제외 |
| 전체 native / 종료 세션 | 38회 / 20세션, 모두 충돌 검사·종료 P 확인 |
| native Instance·8Points·audit 대조 | 실제 112성분 모두 일치 |
| 원형 물리 부모 치수 비교 | 52회, 불일치 0 |
| 선택본의 물리 자산 coverage | 17/19종 |
| 복합 부모 | 8개, 유모차 3성분·반려동물 동반자 2성분 확인 |
| 실제 이동 부모 | 6개, 이동 증거 확인 |
| 같은 cell 내 동일 payload 재취득 | 0 |

첫 4장을 멈춘 뒤 재개해 원문·승인·누적시간을 보존했다. 완료 후 재실행도 새 센서 취득
0회와 원문·선택·누적시간 불변을 확인했다. 해당 runtime의 CPU 회귀는 2,019개 통과,
기존 고정 제외 1개다. 독립 보고서는 `smoke_v3_independent_audit_v1.json`이며 당시 runtime
425개와 입력·native 20,513개 pin이 분석 전후 일치했다.

검토 장면 `03598`의 Bollard04는 LiDAR 0점이다. CAMERA3의 RGB·Instance 마스크에는
대응 물체로 추정되는 blob이 있지만, 동기 카메라 전체의 BBox2D/3D 원문에 해당 UID와
색을 연결하는 행이 없다. 투영과 외형만으로 UID를 확정하거나 GT를 합성하지 않는다.
따라서 실제 사각이라고 단정하지도 않고 `primary_no_sensor_evidence` 검토를 유지한다.
다른 미선택 자산 Scooter_02도 이 장면에 있어 선택본의 19종 coverage를 주장할 수 없다.

현재 alongside 생성은 작은 객체를 Ego rear-axle 기준 정중앙 옆에 매우 좁게 샘플링한다.
도로 위치·yaw를 다시 뽑아도 이 구조는 그대로여서 3개 새 후보도 같은 검토로 끝났다.
기존 간격·side role·phase와 자산 구성은 유지하면서 앞측면으로 종방향 관측을 분산하는
별도 opt-in 후보를 검증한다. 원문 연결 없는 mask를 승인하는 fallback은 추가하지 않는다.

또한 exact-source smoke는 미준비 natural 경로를 포함하면 실제 준비 뒤 payload가 달라진다.
이 구조 불일치는 사전등록 후보의 적격성에서 바로잡되, full5K의 미준비 22개는 유지한다.
원본 exact 검사나 실제 준비 경로의 계보 검사를 완화해 해결하지 않는다.

혼합 overlay의 31장 일괄 입력 변환은 예상대로 거부됐다. 원본 30장과 준비 alias 1장은
별도 cohort로 변환·검증하고, 공통 split으로 최종화됐다고 해석하지 않는다.
실패한 `training_inputs_v3`와 별도 `training_inputs_separate_v1` 결과를 모두 보존한다.
별도 두 cohort의 실제 MMDet train/val 전처리에서는 31장·95+1 GT의 중심·크기·클래스를
확인했다. `loader_separate_v1/report.json`이며 모델 실행이나 학습 성능 검증은 아니다.

## 초근접 관측 위치의 소량 대조

새 정책은 원형 native 높이 1m 이하·가로세로 1.5m 이하의 단일 물리 객체, 기존 정적
alongside·좌우 역할·유효 배치 간격 0.30m 조건에만 명시적으로 적용한다. 전수 후보는 23장이다.
객체 중심을 Ego rear-axle 기준 전방 1.0~1.5m의 측면에 놓되, 실제 좌우 역할과 기존
보수 배치 envelope 간격을 계속 검사한다. 좁은 자산에서 전방 역할로 바뀌는 후보는 거부한다.
기본 tag 없는 800개 좌표·증거·RNG state는 기존 구현과 완전히 같았다.

원본 두 배치와 각 배치의 새 관측 두 개를 사전등록했다. 실제 결과는 원본 2개 모두 같은
가시성 연결 문제로 검토, 새 관측 4개 모두 승인이다. 이는 **두 layout의 조건 대조**이며
독립 표본 4개의 성공률로 일반화하지 않는다. 생성 시 카메라 OBB 투영은 후보 휴리스틱일 뿐,
native annotation과 실제 마스크를 대신하는 승인 조건이 아니다.

새 4개는 동기 BBox2D·BBox3D 원문에 UID8500과 같은 RGBA가 명시됐고 Instance 마스크의
실제 해당 색은 각각 6,172 / 4,891 / 6,591 / 5,316픽셀이었다. LiDAR는 6개 모두 0점이므로
이번 개선은 카메라 근거 확보다. 원형 물리 부모 12개의 8모서리 치수 불일치는 0이며
모든 세션에서 충돌 검사·종료 P를 확인했다. 실제 annotation↔고정 Ego footprint 간격은
약 34~37cm였고, 배치용 proxy의 30cm와 혼동하지 않는다.

`close_primary_diagnostic_v3/registration.json`, `capture_close_primary_diagnostic_v3`,
`close_primary_native_independent_audit_v3.json`에 준비·실제 취득·독립 검산을 보존한다.
UID 연결 없는 과거 두 관측을 소급 승인하거나 GT를 수정하지 않았다.

## v158a 새 원형 구성

후속 source는 `close_primary_fullsource_v1/candidate_v158a/manifest.json`이며 SHA는
`4f7fc3b31e23688b43913f2457d570cf014c19e04b2c2e391b463647809e02c5`다.
동일한 전체 band `[1.0,1.5]m` 정책으로 해당 23장만 새로 생성했다. 진단에서 성공한
payload나 좁은 두 sub-band를 가져다 쓰지 않았다. 직전 v157a 대비 나머지 4,977개 payload
경로·SHA는 그대로이며 기존 logical quota ID·capture split·자산 구성·동작·미션 단계·좌우
역할을 보존했다. 이는 E24를 학습한 기존 완료 5K와 4,977장이 같다는 뜻이 아니다.
변경 spec은 새 정책 tag 한 개뿐이고, 새 frozen-spec 파일과 old→new mapping을 따로 만들었다.

23장은 물리 10종에 분포하며 우측 18/좌측 5장이다. 전체 5,000개 geometry로 공식 공통
overlay를 다시 할당해 train 4,250 / val 750, 5,000개 배치 그룹, 기존 69개 link·316개 셀의
train 전부 포함과 122종 자산별 최소 5개 그룹을 검산했다. capture split은 그대로지만 파생
학습 split은 24개 장면에서 달라졌고 변경 목록을 receipt에 보존했다.

원형 기준 구성 표의 총 부모 11,425개, 물리 거리 2,797/865/282개와 원거리 차량 네 칸,
정적 구역 300/120/180은 모두 유지했다. 개별 물리 자산 거리 칸은 `02924`의 보조 RedBarrel
1개가 20~30→0~20m, Steel_Barricade 1개가 0~20→20~30m로 바뀌었다. 두 변화는 같은
자산 구성·기존 제약 안에서 새 카메라 제안을 찾은 재생성 결과이며 총계를 상쇄해 숨기지 않는다.

`close_primary_fullsource_v1/build_receipt.json`과
`v157_v158_planned_distribution_v1.json`에 실제 전수 비교를 기록한다. 생성·분할 CPU 통과는
native 승인이나 본수집 시작이 아니며, 최종 source 그대로의 별도 32장 검증을 요구한다.

최종 코드의 전체 CPU 회귀는 `regression_v6/receipt.json`에 2,041개 통과·기존 고정 제외
1개로 기록했다. 직전 v5의 실패 1개는 브라우저 시험용 localhost 서버가 sandbox 권한으로
열리지 않은 `EPERM`이며 승인 receipt를 만들지 않았다. 해당 검사를 빼지 않고 로컬 서버
권한으로 전체를 다시 실행해 통과했다. 별도 캐시 정리 도구의 안전 회귀 3개도 통과했다.

## 병렬 디스크 정리

사용자의 추가 요청으로 분석 캐시의 완전 중복 65,770개를 SHA로 대조해 hardlink로 합쳤다.
내용·기존 경로와 보고서는 유지하며 중복 할당 약 6.16GiB를 회수했다. 팀장님 최신 풀주행의
Music 중복 9파일과 ZIP도 canonical 9파일 및 ZIP 전체 entry의 SHA 일치를 재확인한 뒤 지웠다.
원래 bag·영상·metadata는 `rosbags/eval/perception/261014_teammate_fullstack/`에 보존했다.
현재 Base5 원본·native·실패 근거·기존 유효 5K와 고유 분석 bag은 삭제하지 않았다.
두 정리를 합쳐 약 7.3GiB를 확보했다. ZIP의 압축 컨테이너 자체는 삭제했지만 그 안의
9개 파일 내용은 canonical 사본과 모두 일치했다. ZIP 파일 바이트까지 보존했다는 뜻은 아니다.

정리 계획·실행 receipt·보호 대상 검산은 `artifacts/evaluation/base5_cleanup_261017/`에 있다.
원문 링크가 유지되도록 중복 캐시만 합쳤으며 다른 입력을 수정해 hardlink peer까지 바꾸면
안 된다. 이 캐시는 읽기 전용 분석 입력으로 취급한다.

## 최종 검증 결과

`registered_smoke_v4`는 v158a에서 미리 선정한 32장·필수 coverage 45개를 등록했다.
`capture_smoke_v4`는 32/32 승인·검토 0·실패 0, 누적 233.0초로 종료했다. 첫 4장의 중간
재개와 완료 후 무촬영 재실행을 모두 통과했고, 완료 재실행의 추가 native 취득은 0회다.
v157a의 과거 31/32 source-cell·30/32 exact 판정은 소급 변경하지 않는다.

독립 감사에서도 exact payload 32개·고유 SHA 32개, alias·geometry 교체·추가 calibration·
동일 payload 재취득 0을 확인했다. native 104성분의 Instance·8Points·audit와 실제 센서
근거가 일치했고, 물리 19종 43부모의 원형 치수 불일치는 0이었다. 최대 축 차이는 약
0.024mm다. 복합 4종 8부모의 성분과 실제 이동 5장면·5부모(editor 3 / 독립 음영 2)도
정상이며 11세션 모두 충돌 검사와 종료 P를 통과했다. 최초 4장의 raw 132개 파일은
SHA·inode·mtime·크기가 그대로다. `smoke_v4_independent_audit_v1.json`에 runtime 427개·
입력 23,620개와 감사 대상 1,257개 파일의 원문 보존 근거를 기록했다.

`training_inputs_v4/report.json`과 고정 Docker의 `loader_v4/report.json`에서 32장·유효
GT 101개(Vehicle 26 / Pedestrian 26 / Obstacle 49)의 Q90·upright·3클래스·512×1056
입력을 확인했다. 실제 MMDet train/val 전처리에서 중심·크기·클래스·ROI·유한 XYZI·카메라
tensor를 대조했다. 각 관측을 두 전처리 모드로 검사한 것이며 원래 split이나 데이터 수를
바꾸거나 같은 장면을 실제 학습의 train/val 양쪽에 넣은 것은 아니다. 모델·GPU·학습은 미실행이다.

## 동결 승인본과 실행 직전 조건

최종 승인본은 `release_v158a/release.json`이며 SHA는
`0a3dd5c96b4522bc9d0547bed845c97ebb02d228b30ff398e1d7ed688b726180`이다.
현재 source·runtime 427개·입력 pin과 위 검증 receipt를 묶었으며 `bulk_ready=true`,
`bulk_blockers=[]`, `bulk_started=false`, `training_started=false`를 확인했다.

`release_v158a/start_readiness.json`의 읽기 전용 실행 전 검사에서도 런처·입력·분할 계약을
통과했다. 10/17 16:30 KST 기준 여유는 약 59.8GiB로 신규 시작 기준 55GiB를 충족했다.
운영 예산은 5K **8~12시간 / 45GiB + 안전 여유 10GiB**로 유지한다. 최종 소량 32장의
233초를 단순히 5K에 곱하지 않는다. 다른 작업이 공간을 쓰면 실제 시작 시 다시 검사한다.

승인된 런처는 `launch_base5_obstacle_support.py`다. 다음 명령은 명시적인 본수집 승인 뒤에만
실행한다. 이번 검증에서는 실행하지 않았으며 목표 output 디렉터리도 만들지 않았다.

```bash
python3 -u dataset/morai_od/scripts/launch_base5_obstacle_support.py \
  --release artifacts/evaluation/base5_unit_release_261017/release_v158a/release.json \
  --environment sunny13 --run --confirmed-environment sunny13
```

output은 `/home/luv/morai_cap/base5_unit_v158_261017/collection_sunny13_5k`다.
이 런처는 `--run`을 빼면 CPU 준비 모드이며 별도 `--prepare-only` 인자는 없다.
다른 schema용인 `launch_base5_fresh.py`로 바꾸지 않는다.

Sunny13·Capture Mode·카메라 1~4/LiDAR 5 annotation ON·수집 전용 네트워크·단독 제어권을
실제 시작 시 확인한다. 검증한 MORAI는 재시작 후 확대 시나리오에 노출되지 않은 원형 전용
프로세스다. 과거 확대 시나리오를 로드한 뒤 단순히 scale 111로 돌아오는 것만으로 잘못된
native 치수가 초기화된다고 가정하지 않는다. 그런 경우 새 프로세스에서 먼저 원형을 검증한다.

별도 최종 보충의 `completion_live_qualified`와 일반 batch 내부의 제한된 회복은 다른 기능이다.
현재 `completion_live_qualified=false`이므로 별도 `--complete`는 승인하지 않는다. 일반 batch
내부 회복은 위 실제 2/2 대조와 반복 방지 계약을 사용한다. 미준비 음영 이동 22장은 full5K에
유지하며 촬영 전에 개별 terrain/reference 준비를 거친다. 이를 삭제하거나 이미 준비됐다고
간주하지 않는다. 재학습 AP·대회용 4090 풀스택 성능·나머지 다섯 환경 승인은 후속 검증이다.

## 최종 Sunny13 5K 시나리오 확정과 촬영 대기 준비

2026-10-17 17:58 KST, 사용자의 최종 5K 시나리오 준비 요청에 따라 위 v158a를 운영 입력으로
확정했다. 이미 전체 5,000개가 생성·검증된 구성이므로 새 seed나 다른 geometry로 다시 생성하지
않았다. 기존 완료 5K를 복사해 새 배치라고 부른 것도 아니다. 최종 manifest와 release SHA는
위 동결값 그대로이며 과거 16:30의 목표 output 미생성 기록은 당시 상태로 보존한다.

실제 scenario 5,000개를 한 번씩 읽어 고유 ID·고유 payload SHA 각각 5,000개, 선언 SHA
불일치 0개, 물리 object의 원형 배율과 범용 2,650 / 미션 2,350 구성을 확인했다.
공통 overlay의 train 4,250 / val 750·69개 link·316개 셀·122종 train 하한도 일치했다.
위치 파일들은 보존된 source 계보의 여러 디렉터리를 참조하므로 manifest 한 파일만 복사해
독립 전달본으로 간주하거나 원래 source 폴더를 정리하지 않는다.

승인 런처에서 다음 **CPU 준비 명령만** 실행했다. 이 단계는 운영 계약·대기 상태 파일을
만들지만 native 취득·MORAI 환경 변경·동적 22장의 지면 준비는 실행하지 않는다.

```bash
python3 -u dataset/morai_od/scripts/launch_base5_obstacle_support.py \
  --release artifacts/evaluation/base5_unit_release_261017/release_v158a/release.json \
  --environment sunny13
```

준비 output은 `/home/luv/morai_cap/base5_unit_v158_261017/collection_sunny13_5k`다.
`prepared_waiting_for_resume`, requested 5,000 / finished 0 / approved 0 / pending 5,000,
빈 selected ledger, child capture 디렉터리와 센서 취득 근거 0개를 확인했다.
16장 chunk·static-ready·overlap OFF·5,000개 source 계약을 만들었으며 기존 raw는 연결하지 않았다.
당시 여유는 약 59.7GiB로 신규 시작 기준 55GiB를 충족했다. 실제 촬영 시작 때 다시 검사한다.

감사 스크립트와 결과는 artifact root의 `final_sunny13_preparation_v1/audit_source.py`,
`source_audit.json`, `verify_preparation.py`, `prepare_receipt.json`에 보존한다.
새 준비가 과거 32장 native 검증의 대체 실험은 아니며, 수집기 코드는 변경하지 않았다.

실제 촬영 시작·정상 중단 뒤 재개에는 앞 절의 **`--run --confirmed-environment sunny13`**
명령을 사용한다. 취득분이 생긴 output에 위 CPU 준비 명령을 다시 실행하지 않는다.
별도 `--complete` 미승인·음영 이동 22장의 촬영 전 개별 준비 조건은 그대로다.
