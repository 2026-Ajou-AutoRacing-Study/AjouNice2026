# Base5 본수집 직전 최종 확인 — v124 runtime / v123 입력

2026-10-08 12:14 KST 검증 종료 기준이다. 구현 기준은 `1587f300`이며 이번 추가 변경은
읽기 전용 대시보드의 수집 전/중단 표시와 해당 테스트뿐이다. 캡처 runtime/hash는 바뀌지 않았다.
**Sunny13 5K 본수집을 시작할 준비는 됐다. 본수집·학습은 아직 시작하지 않았다.**
운영 준비 통과를 5K 전체 취득 성공, 모든 지형의 접지 인증, 모델 성능 인증으로 해석하지 않는다.

## 고정한 입력과 전수 확인

Release: `/home/luv/morai_cap/base5_v2_261008/regions_5k_v123_release`.
현재 시작 입력은 `environment_suite/sunny13/manifest.json`이다.

- Sunny13 manifest SHA-256:
  `8eb9cd73e0f94e311dbe3cbecbefd5caea5d603782e5b15a81299dd619c23509`.
- nominal `scenarios_5k/manifest.json` SHA-256:
  `9e1e381633a7e28085a73bf6cae7b5fcb5ed439f3dce623c879d9267bc2c51e3`,
  기존 v123 동결본과 동일하다.
- 현재 코드로 실제 시작 manifest의 5000셀/5000고유 payload를 재감사했다.
  `OFFLINE_PASS`, 오류0. 구역·도로 지지 proxy·NPC Park·두점 경로·quota 계약을 검사했다.
  이 오프라인 검사는 물리 엔진/센서 가시성의 전수 실험이 아니다.
- nominal/ordered의 전체 entry 집합이 같고 정적4822→실제 이동178 순서다.
  train4255/val745/test0, 122종 planned primary 최소5회를 확인했다.
- sunny/foggy×11/13/15의6manifest 파일 hash, 전체 entry/order/group/split 동일성,
  환경 tag/weather/hour 및 서로 다른 output을 확인했다. 6개 본수집 output 모두 미생성이다.
- parent군·자산별 계획 노출을 재계산해 v123 스냅샷의10079/60474와 같음을 확인했다.
  원본 스냅샷 시각2026-10-08 11:05:24 KST를 새 취득 결과처럼 바꾸지 않았다.
  비교는 계획 노출 열이며 과거 source 대비 증감 provenance와는 별개다.

현재 구성/거리/자산 정책은 [CURRENT](BASE5_V2_CURRENT.md), 계획 노출 표는
[v123 동결 EVIDENCE](261008_base5_region_release_evidence.md#객체군별-계획-노출--v123-동결-스냅샷)를 따른다.
Base5-only/single-frame/4class/7D/native Instance/velocity off/scale1/ROI±100m·횡±30m는 그대로다.

## 최종 runtime의 실제 MR 확인

별도 진단 root: `/home/luv/morai_cap/base5_v2_261008/prebulk_v124_final`.
v123 release의 같은 사전 선정33셀을 새 output에서 실행했다. 본수집 목록/원본은 변경하지 않았다.
이 진단 raw/infos는 본5K에 섞지 않는다. 기존 pilot도 삭제하거나 새 runtime으로 재개하지 않았다.

| 확인 | 결과 |
|---|---|
| MR 대표 캡처 | 최종33/33 승인, 부족0, 220.626초 |
| 실제 촬영 |34회: 최초32승인/1review + 동일quota 대체1승인 |
| review | 근접 혼합 `b5v2_00082`의 Woman5 보조 객체 누락, 원본 보존 |
| 실제 이동 | 사거리 Girl1 달리기/worker_m_1 걷기2/2, 캡처창 이동·끝점 gate 통과 |
| CPU 변환 |33프레임(train30/val3), GT72(vehicle43/obstacle16/pedestrian8/animal5) |
| 가시성 예외 유지 |빈 배경1, camera-only Obstacle1 유지; 미가시 secondary는 GT에 추가하지 않음 |
| CPU 파일/좌표 확인 |4camera/4feature LiDAR/calibration/7D 검사 오류0 |
| 안전 종료 |10 child/recovery의 종료·경계 Park 확인, unsafe0 |
| 완료 ordinary resume |1182 raw의 SHA/inode/mtime/size 불변, 10 event 로그/선택 내용/누적시간 불변, 추가capture0 |
| 기존 v124 보충 결과 재개 |최초1+보충1의 승인2 유지, raw72파일 불변, 추가capture0 |
| 자동 회귀 |370개 PASS /75.407초 |

33셀은 로터리·고주로 앞뒤 거리·범용 단일/혼합·근접·사거리 이동·희귀 자산·음영 접근·배경의
coverage-oriented 표본이다. 이번 이동2셀은 사거리이며 음영24개 전체 이동 인증이 아니다.
누락 review에서 child가 안전 종료된 뒤 parent가 recovery를 수행했다. 이를 통신/Park 안전 halt를
무시하고 강제 재개한 결과로 해석하지 않는다. 시험용 접촉 주입은 이번33셀에 사용하지 않았다.
이전 보충 fault-injection의 범위는 [v124 보충 EVIDENCE](261008_base5_completed_replenishment_evidence.md)에 둔다.

산출물: `offline_audit.json`, `planned_exposure.json`, `live_pilot/`, `infos/`, `readiness.json`,
`completed_resume_verification.json`, `replenishment_resume_verification.json`, `cost_report.json`.
후보 생성/GT/production detector/threshold/tracker/planning/control/localization은 이번에 변경하지 않았다.

## 통신·공간·시간·대시보드

- 현재 MR/master/guarded bridge만 사용한다. 시작 전3초 수신 검사에서 Ego50.07/Object49.84/
  Collision49.83Hz, 최신 메시지 age≤0.042초, 실제 Ego 속도≈0, collision peer 없음,
  `/Service_MoraiEventCmd` 등록을 확인했다. 이후 실제 load/capture/control도33셀에서 동작했다.
- 디스크 여유 약247.8GiB다. 이번 표본의 QA/review 포함 외삽은5K 약26.49GiB,
  30K 약158.93GiB다. 변환 캐시/압축 복제/부족분 비용은 별도이며 자동 삭제하지 않는다.
- 정적31셀6.626초/승인, 사거리 이동2셀7.604초/승인이다. 가중 단순 외삽9.252h/5K지만
  음영24개의 독립 지면 준비를 같은 비용으로 인증하지 않는다. 운영 예산은 **10–13h/5K**,
  **60–78h/6환경**을 유지하고 본수집 첫100–200셀의 실제 누적rate로 갱신한다.
- 읽기 전용 대시보드를 현재 본수집 root/localhost8075로 실행하고 HTTP를 확인했다.
  지금은 `not_started`/`수집 전`이며 root를 만들거나 촬영하지 않는다. 같은 포트에 중복 실행하지 않는다.
  기존8081은 과거10K output용이므로 현재 진행률이 아니다.
- 표시 버그를 수정했다: `not_started`를 갱신 지연/완료로 표시하지 않는다.
  ETA0/finished=requested만으로 running/halted를 완료라고 표시하지 않으며,
  중단/일시정지/부족분/진짜 완료를 별도로 표시한다. 회귀 테스트를 추가했다.

## 시작 이후 운영 경계

다음은 사용자 시작 승인 후 [런북](261007_base5_5k_capture_runbook.md)의 터미널3 명령이다.
기존 master/bridge/8075는 살아 있다면 재실행하지 않는다. MR UI에서 Capture Mode/Sunny13/대회 센서를
다시 확인한다. manifest는 실제 날씨/시간을 자동 설정하지 않으므로 파일 검사를 UI 확인으로 대체하지 않는다.

- 성공 raw/원본 manifest를 보존하고 runtime/config/source를 동결한다.
- 안전한 품질 실패는 기존 bounded recovery→부족분 기록→다음 셀이다.
  **통신/Park/실제 정지 불확실은 여전히 중단**한다. 실패 전부를 억지 승인하지 않는다.
- 완료 후 같은 명령의 `--resume --replenish --replenish-rounds 2`로 부족 원래 셀만 보충한다.
  누적 상한은 최대3, ordinary 완료resume은 검증만 하며 성공분을 다시 촬영하지 않는다.
- 이후 실제 visible-primary/거리/방향 coverage와 CPU 변환을 확인한다.
  소량 표본의33/33을 전체5000/5000 보장이나122종 실제 GT 충족으로 주장하지 않는다.

추가 높이 개발/새 모델 추론을 본수집 시작 조건으로 남겨두지 않는다. 현재 알려진 음영 이동의
지원 근거 부족은 이미 bounded 부족분 정책 안에 있으며, 데이터 품질 기준을 낮추지 않고 결과를 보고한다.
