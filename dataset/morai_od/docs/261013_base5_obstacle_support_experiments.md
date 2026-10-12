# Base5 장애물 배치 보강 실험

문서 상태: 구현과 실험 근거. 기준일: 2026-10-13.

완료 Sunny13 5K는 보존하고 새 경로에서 생성·소량 캡처·native GT 분석을 수행한다.
5K 본수집은 실행하지 않는다. 현재 MORAI UI는 사용자가 연결한 Sunny13이며 다른 날씨의
실제 가시성을 검증한 것으로 해석하지 않는다.

## 입력과 보존

기존 학습 입력은 `artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release/infos/512x1056`의
train 4,255장과 val 745장이다. 클래스 원뜻과 parent/component를 보존하며 이번 실험에서는
threshold·checkpoint·모델 구조·이미지 입력·학습 split을 바꾸지 않는다.

기존 다음 환경의 `fresh_suite_final_v142`는 baseline으로 보존한다.
실험 root는 `~/morai_cap/base5_obstacle_improvement_261013`이다. `final_v149/sunny13`은
seed 261013의 독립 Ego/객체 geometry를 새로 생성한 5,000개 배치 계획이지 새로 찍은 5,000장이 아니다.
완료 Sunny13를 덮어쓰거나 30K 계획에 추가 5K를 자동 합산하지 않는다.

## 기존 센서 지원

현재 512 입력의 실제 train GT에서 20~30m Obstacle은 610개이고 LiDAR 20점 이상은 27개다.
이 가운데 primary 118개와 secondary 492개를 구분한다. 0~20m 노출과 20~30m 지원 부족은
동일 문제가 아니므로 원거리 객체를 일괄 근거리로 옮기지 않는다.

거리별 수치의 분모는 GT 관측이다. LiDAR 20점은 분석 구간이지 새 승인 조건이나 정확도 지표가 아니다.
camera-only 객체와 일부만 보이는 GT도 기존 증거 정책에 따라 유지한다.
43장 공통 승인 비교의 20~30m 장애물 GT는 21→24개지만 LiDAR 20점 이상은 양쪽 모두 0개다.
보강으로 중거리의 희소한 센서 정보 자체가 해결됐다고 주장하지 않는다. 근거리에서 형상·위치를 배우는
사례를 늘리면서 중거리·원거리의 기존 지원을 유지하는 선택이며 실제 AP 효과는 재학습 후 평가한다.

## 후보 변경

| 후보 | 변경 | 판단 |
|---|---|---|
| 초기 재배치 | 범용 혼합의 일부 보조 장애물 거리만 조정 | 같은 객체 수 165개를 배치한 32장 비교에서 LiDAR 20점 이상 장애물 GT 44→48개. 두 쪽 모두 32장 승인 |
| 근거리 70% 재배치 | 기존 20m 이상 보조 장애물을 대부분 근거리로 이동 | full 5K offline에서 20~30m 물리 장애물 배치가 774→463개로 감소. 중거리 지원을 보강할 목적과 충돌하므로 채택하지 않음 |
| rich floor | 기존 근거리·중거리 보조 장애물을 보호하고 30m 이상 보조만 재배치. 일부 범용 혼합에 장애물 하나 추가 | 공통 승인 43장 비교에서 근거리 GT 64→98, LiDAR 20점 이상 39→55개. Vehicle 36개와 Pedestrian 69개 유지 |

43장은 두 실행에서 승인된 동일 원본 ID를 맞춘 비교다. 재시도로 Ego/객체 위치가 달라진 장면도 있으므로
모든 센서 pose를 완전히 고정한 인과 실험이나 재학습 정확도 개선으로 해석하지 않는다.

`generic-obstacle-rich-floor-v145`는 이미 움직이는 객체가 없는 범용 regular/dense 혼합만 대상으로 한다.
미션·고주로·로터리·음영·배경·generic single은 보호한다. primary 종류·거리 cell·방향과 원래
모든 actor의 종류를 보존한다. 원래 8개 상한에 도달한 장면은 추가하지 않는다.

추가는 seed의 결정적 선택으로 약 60%의 eligible 혼합에서 최대 하나다. 19종의 물리 장애물을
순환 할당하며 footprint cost 2~6 범위로 제한한다. 비용은 실제 정적 배치 제원으로 계산한다.
추가한 비용만큼 해당 spec의 budget을 명시적으로 늘리고 원래 RNG budget도 별도로 보존한다.
가중치 없는 소형 자산만 대량 추가하거나 기존 사람·차량을 장애물로 교체하지 않는다.

추가 객체는 작은 자산 중심 6~18m, 큰 footprint는 8~20m에서 전후좌우로 배치한다.
높이 35cm 미만은 6~10m, 75cm 미만은 6~14m로 제한해 작은 객체의 유효 이미지 크기를 확보한다.
새 보조 객체가 primary의 수평 시야를 85% 이상 덮는 footprint proxy는 CPU에서 거부한다.
이 검사는 실제 카메라 가림의 인증이 아니며 기존 장면의 가림을 일괄 제거하지 않는다.
기존 30m 이상 보조 장애물은 근거리 또는 20~29m로 이동할 수 있지만 primary의 원거리 quota는 유지한다.
기존 0~20m와 20~30m 보조 장애물은 재시도에서도 해당 거리 구간을 벗어나지 못한다.
새 budget·전체 data ID 목록·추가 ordinal·거리 구간을 spec에 동결한다.

## 새 Sunny13 배치 계획

독립 geometry의 baseline과 rich floor 계획을 비교한 offline 수치는 다음과 같다.

| 계획된 parent 배치 | baseline | rich floor |
|---|---:|---:|
| 전체 actor | 10,753 | 11,425 |
| 물리 장애물 | 3,272 | 3,944 |
| 물리 장애물 중심 0~20m | 1,935 | 2,785 |
| 물리 장애물 중심 20~30m | 753 | 863 |
| 물리 장애물 중심 30m 이상 | 584 | 296 |

672개의 추가 장애물이 19종에 35~36회씩 배분됐다. 객체 이름·배치 수가 실제 유효 GT 수와 같다는
보장은 없다. 기존 5K의 등장 횟수와 이 새 seed의 계획 수를 같은 관측으로 혼합하지 않는다.
기존 actor 목록·primary 계약·split/group·frozen specs를 감사했고 오류 0개다.
변경 대상이 아닌 4,231개 payload는 baseline과 SHA가 동일하다.

## 검증과 남은 일

기본 정책을 유지한 통신·Park 확인, 32장 대조 실험, 더 넓은 범용 baseline 48장,
단일 자산 96장의 거리·yaw 진단을 수행했다. baseline 48장은 승인 47장/부족 1장이다.
가시성·spawn 누락·심각한 높이 오류는 원본을 보존하고 기존 유한 재시도·QA로 처리한다.

학습 GT 분석은 현재 512×1056의 실제 instance mask를 다시 측정한다. 이전 320 crop의 가시성을
그대로 풍부한 사례로 부르지 않는다. LiDAR 0/1~4/5~19/20점 이상과 camera gate를 분리한다.

새 Sunny13 geometry의 정적 회귀 64장은 61장 승인/3장 부족으로 종료했다. 실제 보충은
첫 새 후보로 2장 회복했고 남은 한 장은 총 3회 후 명시적인 부족으로 종료했다. 완료 후 resume은
기존 native intensity 83개와 두 ledger의 SHA를 유지했고 추가 촬영 0개였다.
보호된 실제 이동 회귀 8장은 모두 승인됐다. 마지막 합계는 정적 63/64와 이동 8/8이며
이를 64/64 또는 5K 수집 완료로 바꾸어 보고하지 않는다.

96장 진단은 CargoBox·PE_Drum·WoodBox·TrafficBarrel·Steel_Barricade·concrete-barrier·Bollard04·Scooter_01을
10/20/25/35m, 세 연속 yaw sector에서 확인했다. 모두 native QA를 통과했다.
35m의 CargoBox·PE_Drum·TrafficBarrel·Bollard04에는 LiDAR 0점인 사례가 있었지만 현재
512 입력의 instance mask 조건을 충족했다. 원거리 장애물을 일괄 삭제할 근거가 아니다.

48장 파생 검사는 native full RPY의 upright enclosure와 512 mask 가시성을 적용했고
Q90 JPEG portable package를 검증했다. 카메라 RGB만 172,241,589→15,839,600 bytes로 바뀌었으며
mask·depth·native annotation·LiDAR·metadata는 무손실 보존했다. 시험 package는 본학습에 합치지 않는다.

### 재시도 경로 보완

기존 replacement는 정적 제원 적용 과정에서 만든 얕은 context 복사에 지도/region index를 생성했다.
재시도마다 이를 다시 만드는 병목이 나타났다. 새 정책의 replacement는 원본 context를 먼저 prewarm한다.
또한 재시도의 보호된 근거리·중거리 객체는 해당 구간에서 직접 샘플링한다. 기존 장면 승인 조건은 바꾸지 않았다.

512회 offline proposal을 소진한 경우에는 이전의 거부된 native frame과 실제 Park 근거를 검증하고
새 시나리오를 로드하지 않은 부족분으로 기록한다. 일반 ValueError, hash 변경, 통신·제어 불확실은
계속 전체 중단한다. 빈 report를 Park 근거로 사용하지 않으며 이전 report SHA와 실제 child session을 재검사한다.

### 남은 긴 NPC 배치와 시뮬레이터 종료

음영 구간의 `2016_Hyundai_Universe`를 도로에 거의 수직으로 놓은 장면은 NPC가 실제 이동했다.
첫 보충에서 Object position이 before→after 약 0.69m 움직였고 계획 pivot과 최대 2.08m 차이를 보였다.
이는 고정된 pivot 오프셋만으로 설명할 수 없으며 기존 정지 검사로 올바르게 거부됐다.
12m 차체가 도로 측면 지지 범위를 벗어났을 가능성을 시험하기 위해 같은 자산·미션의
road-aligned 4장 후보를 `long_vehicle_probe4`에 생성했지만 실제 캡처는 아직 하지 못했다.
이를 이유로 버스를 제외하거나 정지 검사 25cm를 완화하지 않는다.

저장된 전방 RGB에서도 해당 버스가 음영 구간 양쪽 측벽에 걸쳐 있고 차체가 기울어진 모습이 보인다.
계획 metadata의 중심 높이는 사용할 수 있었지만 네 모서리는 모두 `roadside_estimate`였으며
일부는 가장 가까운 road link에서 약 6m 떨어져 있었다. `footprint.all_supported=true`는
모든 점의 **높이 추정 가능**을 뜻하지 실제 아스팔트·타이어 접지 인증을 뜻하지 않는다.

새 5K를 read-only로 감사한 `parked_npc_support_sunny13.json`에는 정지 NPC actor 4,260개,
두 모서리 이상에서 도로 지지를 추정하지 못한 후보 65개가 있다. 긴 NPC 359개 중 해당 후보는 16개다.
이는 map 선별 신호이고 실패율이 아니므로 65개를 자동 삭제하지 않는다. road-aligned 4장 후보는
이 선별 신호가 없지만 native 실제 정지·접지는 아직 확인하지 않았다. 생성·학습 GT는 변경하지 않았다.

추가 대조 실험의 `sunny13_sunny11_b5v2_01122` 로드 뒤 Simulator가 종료됐다.
Player.log의 마지막 스택은 `signo:11`과 Vulkan descriptor cache / `GfxDeviceVKBase::DrawBuffers`다.
생성기 파일 오류나 OOM으로 확정하지 않는다. 직전에는 NPC waypoint index 예외도 기록됐다.
수집기는 telemetry stale·최종 Park 불확실로 halted했고 추가 로드는 하지 않았다.
`matched_baseline64`는 17장 검사 시점에 중단된 안전 실패 근거이며 성공 표본이나 비용 비교에 합치지 않는다.
재시작으로 로그가 덮어써지는 것을 막기 위해 `simulator_crash_261013_Player.log`를 실험 root에 보존했다.
원본과 복사본의 SHA256은 `489759a2f40aa9036ffb74863594f89aa9ace46d11dbb933dd789adf33632264`로 동일하다.

따라서 동결 release는 `bulk_ready=false`다. 일반 배치 보강과 복구 경로의 근거는 확보했지만,
MORAI를 재연결하고 긴 NPC의 4장 후보를 실제 확인한 뒤 본수집 가능 여부를 결정한다.
halted 대조 output을 강제 resume하지 않는다. 재학습 AP와 Foggy 성능은 이번 배치 검증만으로 인증하지 않는다.

## 다음 환경과 비용

새 다섯 환경은 `final_v149/future_suite`에 25,000개의 offline 배치로 생성했다.
각 환경은 독립 geometry이며 원래 primary·actor 목록·미션·split을 유지한다.
물리 장애물 근거리 배치는 환경별 43~45% 늘고 20~30m 배치는 감소하지 않았다.
기존 Sunny13를 더해 총 30K 계획을 유지하며 별도 새 Sunny13 preview를 자동 합산하지 않는다.

정적 64장과 실제 이동 8장의 QA·실패 raw를 포함한 group 가중 예상은 5K 약 11.37시간/40.87GiB다.
이 표본은 어려운 strata를 포함하므로 전체 5K의 확정 비용이나 기존 대비 느려진 원인의 인증이 아니다.
현재 보수 운영 예산은 10~13시간/약 51GiB이며 실제 본수집 초반 rate로 갱신한다.
현재 여유 약 113GiB로는 다른 환경 25K 전체를 동시에 보관하기 부족하다. 완료 데이터 자동 삭제는 하지 않는다.

CPU 사전 확인과 남은 live 절차는 [보강 런북](261013_base5_obstacle_support_runbook.md)을 따른다.

## 최종 코드 검증

- `python3 -m unittest discover -s dataset/morai_od/tests`: 684개 통과.
- scripts/tests의 `compileall`, `git diff --check`: 통과.
- 동결 후보의 CPU prepare-only: 통과, 승인·촬영 0장.
- 동일 후보의 bulk 진입 gate: `bulk_ready=false`로 실행 전 거부됨. native 촬영 없음.
- 기존 Sunny13 selected ledger SHA256: `026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43` 유지.

실험 root의 native·거부된 raw·진단 package·새 5K 및 다음 25K 배치 후보는 git 밖에서 보존한다.
커밋에는 코드·검사·현재 상태·근거 문서만 넣고 planning validator나 모델 배포 설정은 변경하지 않는다.
