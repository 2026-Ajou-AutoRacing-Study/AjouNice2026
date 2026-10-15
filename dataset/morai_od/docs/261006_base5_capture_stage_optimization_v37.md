# Base5 v37 — 캡처 비용 분리와 안전 조건부 대기 단축

상태: EVIDENCE / CURRENT 코드. 2026-10-06. 10K 본 수집·학습은 미시작이다.

## 결과

동일6개 scenario payload를 순차 A/B했다. highway/obstacle/roundabout/background 정적4개,
walking/running2개다. 각 실험6/6 accepted, 촬영6회, deficit0, 모든 종료Park 확인.
원본32셀 v36의 비용 근거는 [이전 비용 측정](261006_base5_capture_capacity_v36.md)을 유지한다.

| 항목 (6셀 합계) | 기존 고정 준비 대기 | 조건부 준비 대기 + 중복 작업 제거 |
| --- | ---: | ---: |
| 캠페인 전체 | 56.556초 | 50.732초 |
| 사전 연결·정지 확인 | 7.878초 | 2.053초 |
| 로드 위치 확인 | 8.180초 | 8.230초 |
| 제어 준비 (service discovery/고정 대기/응답 포함) | 12.698초 | 12.687초 |
| 움직임·목표 도착 확인 | 13.856초 | 13.839초 |
| 파일 생성·0.5초 안정화 | 7.090초 | 7.142초 |
| 출처·충돌·센서/지면 감사 | 0.583초 | 0.587초 |
| 종료Park | 0.061초 | 0.076초 |
| 나머지 child 작업 | 1.369초 | 1.349초 |
| child 외 비용 | 4.842초 | 4.768초 |

실제 전체시간 절감10.30%, 거의 전부 preflight1.313→0.342초/셀에서 발생했다.
센서/지면 감사는 이 작은 live표본에서는 감소가 확인되지 않았다.
로드/제어 준비 단계는 응답시간만 측정한 것이 아니다. 내부 MORAI 병목이나 특정 service 원인을 이 표만으로 단정하지 않는다.
같은 시나리오를 다시 로드했지만 실제 동작 자세/센서 원본까지 동일하다는 뜻은 아니다.

## 변경한 것 / 유지한 것

- 각 child에 monotonic 단계별 timing.json을 남긴다. 계측 자체는 제어·GT를 변경하지 않는다.
- 고정1.3초 preflight 대신, 실제3개 publisher 연결 + Ego/Object/CollisionData freshness +
  최소0.3초 brake 유지 및 정지 샘플을 확인한 경우만 통과한다. 샘플은 유한 값,
  속도0.02m/s 미만, XY span0.02m 미만, Z span0.02m 미만을 요구한다.
  3초 안에 증거가 확보되지 않으면 ScenarioLoad 전에 중단한다. 무조건 대기를 없앤 것이 아니다.
- 같은 capture의4-camera Instance PNG를 객체마다 재디코딩하지 않고 로컬 cache를 공유한다.
  cache는 audit 호출1회에만 유효하며 다음 scene에 재사용하지 않는다.
- 캠페인의 single-cell/deficit manifest를 만들 때10K scenes 전체를 deepcopy하지 않는다.
  선택행과 공통설정은 독립 복사해 원본 변경/aliasing은 금지한다.
- 로드 뒤 위치 확인, 네트워크 settle0.7초, 제어 응답 확인, Drive 확인,
  움직임/목표 도착, 파일 안정화0.5초, 충돌·지면·Z·가시성 gate, 재시도 상한, 종료Park는 유지한다.
- raw GT/학습정책/분포/모델/threshold/tracker/planning/localization은 변경하지 않는다.

## Offline 정확 일치 검증

v36 통과32개 원본 각각에서 cache on/off를 순서 교대로2회 비교했다.
**32/32 전체 audit dict가 정확히 동일**했다. 감사시간 합계(각셀 median)1.828→1.522초,
약16.8% 감사비용 절감이다. 전체수집16.8% 개선이라는 뜻은 아니다.

canonical10K manifest에서 같은 subset 출력을 비교한10회 median은
전체 deepcopy66.584ms → 선택행 deepcopy0.163ms다. 셀마다 scene+deficit deck을
만들 때 반복되는 CPU 비용을 줄이며, live6개에서는10K 배열 비용을 측정하지 않았다.
ledger 전체 재기록 등 다른 장시간 scale 비용은 여전히 남아 있다.

Base5 tests100개PASS. 정지 witness의 stale/짧은trace/이동/높이변화/NaN 거부,
image cache decode1회+출력일치,10K subset 비공유, 기존 STOP/resume/unsafe halt 회귀를 확인했다.

## 원본 / 재현

- baseline: $HOME/morai_cap/base5_v2_261006/timing_baseline_v37
- optimized: $HOME/morai_cap/base5_v2_261006/timing_optimized_v37
- stage 비교: $HOME/morai_cap/base5_v2_261006/capture_stage_comparison_v37.json
- offline cache/manifest 비교: $HOME/morai_cap/base5_v2_261006/audit_cache_benchmark_v37.json
- source: capacity_pilot_v36/manifest.json의 indices0,3,5,9,12,19

```bash
python3 dataset/morai_od/scripts/report_base5_capture_stages.py \
  --before "$HOME/morai_cap/base5_v2_261006/timing_baseline_v37" \
  --after "$HOME/morai_cap/base5_v2_261006/timing_optimized_v37" \
  --output /tmp/base5_stage_report_new.json
```

원 근거를 덮어쓰지 않는다. report는MR에 연결하거나 root status를 수정하지 않는다.
이번 capture 코드/안전계약 hash 변경으로 **v36 및 v37 baseline 세션은 현재 코드로 resume하지 않는다**.
완료 raw는 유지하며 read-only 보고서로 읽는다. 기존 계약파일을 새 hash로 고치지 않는다.
새 본 수집은 새output에서만 시작한다.

## 다음 판단

작은 순차 실험이라 장시간10K 속도와 모든 자산의 안정성을 보장하지 않는다.
10K를 하룻밤에 끝낼 정도의 개선은 아니다. 27.5시간 외삽을 확정적으로24.7시간이라고
바꾸지 않는다. 현 조건에서 대략 하루 규모 수집이라는 판단은 유지한다.
더 큰 개선 후보는 매셀 process/ROS/network 재설정의 반복 제거지만,
네트워크 무재로드나 persistent worker는 별도의 안전·stop/resume·서비스 생명주기 검증이 필요하다.
이번에는 그 구조변경을 구현하지 않았다. 준비시간이 변하면 동적 도착/외관 gate도 함께 재확인해야 한다.
