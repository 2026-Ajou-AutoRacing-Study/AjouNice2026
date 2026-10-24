# Base5 v36 — 대표 32셀 실제 캡처 비용

상태: EVIDENCE. 2026-10-06. 학습/추론 속도가 아닌 MR 동기 캡처 비용이다.
본10K 캡처와 학습은 미시작이다. 생성 완료 canonical은 v35를 유지한다.

## 결과

- 선택32셀 / 통과32 / 부족0. 총314.657초, 통과 셀당9.833초.
- 실제촬영36회: initial33, 같은셀 재배치1, 독립 empty terrain1, 입력Z 보정 재촬영1.
- 보완이 필요했던 원래 셀은3/32(9.375%). 추가촬영4/32(12.5%)는 재시도율과 다른 값이다.
- 모든 촬영 종료 Park 확인. unsafe child0. 통과는 현재 충돌/가시성/지면 등 quality gate 기준이며 모델 정확도 인증이 아니다.
- 원본·실패시도·로그 합계187,994,476bytes(179.3MiB).
- 현재 family별 실측 비용을10K quota로 가중하면 **27.53시간**.
- 저장량 단순외삽은 **54.71GiB**. 데이터 변환/전달 압축본은 별도 용량이다.
- 당시 디스크 여유267GiB. 원본 예산70–80GiB는 여유치이지 실측 상한이 아니다.

| Family | 통과셀 | 실제촬영 | 셀당 child 처리시간(초) |
| --- | ---: | ---: | ---: |
| highway | 10 | 10 | 6.217 |
| roundabout | 2 | 2 | 6.156 |
| other_vehicle | 1 | 1 | 5.990 |
| obstacle | 9 | 12 | 9.818 |
| pedestrian_related | 8 | 9 | 11.812 |
| background | 2 | 2 | 6.208 |

표 시간은 child의 준비·캡처·검증·종료 포함 시간이다. 총314.657초에는 부모 프로세스/quality loop 비용도 포함한다.
child 합계275.748초에 부모 비용을 전체 비율로 배분해10K 시간을 추정했다.
센서 저장/ScenarioLoad/Park 각각의 세부 프로파일링 결과로 해석하지 않는다.

## 실제 보완 사례

- b5v2_00008: primary_no_sensor_evidence → 동일 quota 조건 재배치 → 통과.
- b5v2_00047: actor_ground_plane_unverified → 독립 지면 캡처. 첫 round는 unresolved이며 빈 지면을 학습 정답으로 선택하지 않았다. 두 번째 round의 객체 캡처가 통과해 quota1장만 인정했다.
- b5v2_00925: actor_annotation_above_ground → 입력Z 보정 후 재촬영 → 통과.
- 실패 원본과 보완 근거를 그대로 보존했다. 지면 gate를 완화하거나 raw GT를 수동 이동하지 않았다.

## 경로와 재현

- 원본10K manifest: $HOME/morai_cap/base5_v2_261006/scenarios_10k_v35/manifest.json
- 대표 manifest: $HOME/morai_cap/base5_v2_261006/capacity_pilot_v36/manifest.json
- capture root: $HOME/morai_cap/base5_v2_261006/capacity_capture_v36
- 비용 요약: $HOME/morai_cap/base5_v2_261006/capacity_report_v36.json
- 대시보드: http://127.0.0.1:8076 (로컬 프로세스 실행 중일 때만 유효)

```bash
python3 dataset/morai_od/scripts/report_base5_capacity_pilot.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/capacity_capture_v36" \
  --manifest "$HOME/morai_cap/base5_v2_261006/capacity_pilot_v36/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261006/capacity_report_v36.json"
```

완료 후 read-only resume은 root status elapsed를 다시 기록하므로 비용 보고서는 **resume 전에 생성**해야 한다. 위 보고서를 원 측정 근거로 보존한다.

## 제한 / 다음

family quota를 대략 맞추되 어려운 조건 coverage를 우선한 작은 표본이다.
27.53시간은 보장 시간이나 통계적 신뢰구간이 아니며 전체122종/장시간 누적 지연을 검증한 결과도 아니다.
현재 구현으로 하룻밤10K 완료를 약속할 수 없다.
다음은 준비·프로세스·검증 비용을 분리 계측해 안전장치를 유지하며 줄일 여지가 있는지 확인하거나,
현재 속도를 받아들여 하루 이상 걸리는 본 수집을 시작하는 선택이다.
실패 gate를 제거해 처리량을 높이지 않는다. 아직10K 실행은 하지 않았다.
