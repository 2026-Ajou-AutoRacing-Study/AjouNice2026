# Base5 Sunny13 — 부족 셀의 유연한 대체

2026-10-10. 사용자는 v139의 남은 26셀(0.52%)에 대해 정확한 자산/거리/객체 수 복원 대신
같은 패밀리의 정상 새 장면으로 대체하는 것을 승인했다. 기존 승인 4,974장과 실패 원본을 보존한다.
이 문서는 이전 strict same-cell 보충의 이후 단계이며 QA 완화나 기존 journal 수정이 아니다.

## 계약

- 새 출력: `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release/collection_5k_sunny13_flexible_v140`.
- 유지: Sunny13 환경, 패밀리, 미션/범용 맥락, primary의 의미적 asset pool,
  train/val split, 실제 정적/동적 구분 및 walking/running별 최소 이동 인원.
- 변경 허용: 자산 정체성, 거리 bin, 방향, 관측 phase, 객체 수와 보조 객체 구성.
  고주로/로터리 차량 전용·실제 도로/지면 초기 배치 정책은 원래 생성기가 유지한다.
- 원래 승인 장면의 spec을 donor로 사용하되 **새 seed/geometry를 다시 생성**한다.
  기존 raw나 성공 시나리오를 복사하여 26장으로 세지 않는다. donor 자산 종류는 균등 표본화하고
  실패 primary와 다른 자산이 있으면 우선한다. 가능한 ordinary 관측 거리 10–60m donor를 우선한다.
- 한 후보는 최대 512 proposal, 유효 8후보를 proxy로 비교한다. proxy는 승인 근거가 아니다.
  실제 native GT/가시성·ground·움직임·Ego 정지·CollisionData·Park 검사를 그대로 사용한다.
- 동적 부족 02912는 standing 경찰을 반복할 필요가 없다. 다른 사람 자산이 실제 walking하는
  사거리 장면으로 바꿀 수 있다. 이동을 전부 standing으로 바꾸어 동적 셀을 채우지는 않는다.
- 각 child는 **자기 새 spec의 독립 계약**을 갖는다. 내부 quality 대체 뒤에도 새 계약과
  원래 패밀리/실제 이동 조건을 재검사한다. 원래 failed spec과 같다고 거짓 기록하지 않는다.
- 원래 논리 slot/group와 split은 유지하며 새 `id`에는 `_fxNN`을 붙인다. ledger에는
  `source_cell_id`, `flexible_completion`, 원래→새 자산·수·거리·방향·이동 종류가 기록된다.
- 셀당 최대 5새후보·기본 30분 안전 경계 pause다. 정상 성공은 건너뛰며 완료 resume은 촬영 0이다.
  증거 없는 중단 child/통신/Park 실패는 자동으로 건너뛰거나 강제 resume하지 않는다.
- v139/이전 root의 raw·source manifest·journal은 불변이다. driver와 runtime SHA도 resume 계약이다.

## 실행·최종 선별

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_flexible_completion.py --resume --run
```

실제 전체 승인 뒤 촬영 없이 명세·coverage·CPU infos를 정리한다.

```bash
ajou
python3 -u dataset/morai_od/scripts/finalize_base5_flexible_selection.py
```

최종 `selected_ledger.jsonl` 하나에 기존 4,974장이 이미 포함된다. 과거 ledger를 붙이지 않는다.
`replacement_mapping.json`은 새 26장의 변경 내역이고, `adopted_manifest.json`은 최종 선택의
실제 자산/spec/시나리오/native metadata를 가리키는 **명시적으로 수정한 계획**이다.
기존 Sunny13 manifest를 이 파일로 덮어쓰지 않는다. coverage는 이 새 명세와 선택을 비교한다.
기존 명세로 primary identity를 비교하면 의도적인 대체를 오류로 판정하므로 두 계약을 구분한다.

CPU 변환은 원본 BBox/component/이미지/point cloud를 수정하지 않는다. 현재 320 crop 가시성 기반
임시 학습 infos이며 최종 crop 확정 후 raw에서 다시 변환한다. 모델 학습·추론·production 변경은 하지 않는다.

## 최종 결과 — 2026-10-10 / v140

**4,974장 보존 + 새 대체 26장 = 유효 5,000장, 부족 0장, stage `complete`다.**
26개 모두 첫 후보에서 원래 QA를 통과했다. 정적 25개/실제 walking1개, 추가 active 시간
567.915초(약 9분 28초)이며 이전 전체 본 수집/보충 시간이 아니다. 다른 조건과 시간 단축률의
엄밀한 A/B로 해석하지 않는다. 실패 셀의 exact 복원 대신 새 장면을 승인한 명시적 계획 변경이다.

| 확인 항목 | 결과 |
| --- | --- |
| 기존 승인 보존 | 4,974행 전부 그대로 유지, 원래 raw/source/부모 hash 불변 |
| 새 장면 | 26시도/26 승인/재시도 0, shadow6·obstacle15·highway3·pedestrian_related2 |
| 실제 외관 | 전체 정적 4,822 / 실제 이동 178, 이동 셀을 정지로 대체하지 않음 |
| 새 raw | 26캡처, 새끼리 같은 payload0 / 기존 선택 payload 재사용 0 |
| 자산 변경 | 25셀의 primary 이름 변경, 나머지 1셀은 Pet1의 새 거리·구성·방향 관측 |
| 최소 coverage | 122종 accepted-visible primary floor 결손 0, adopted coverage `PASS` |
| CPU split | train 4,255 / val 745 / test 0 |
| 유효 GT | 10,959 (Vehicle4,513 / Pedestrian2,937 / Obstacle3,239 / Animal270) |
| 파일·중복 | 입력 25,000개 누락 0 / token5,000 unique / scene-group split 중복 0 |
| 종료·resume | 모든 child Park 확인, 완료 `--resume --run` 추가 시도 0·raw0·SHA/시간 불변 |
| 단위 검사 | 전체 507개 PASS: 유연 대체 12 + 최종 변환 5 포함 |

예: 고주로 DEFAULT_BUS 6–10m 후방 셀은 Prius 30–45m 후방 새 배치로 대체했다.
Pet1초근접 혼합 5 actor는 같은 Pet1 포함 15–30m 후방 3 actor로 대체했다.
02912의 standing police1 + walking Man6는 사거리 Woman6 walking1 actor/20–50m 전방으로 대체했다.
원래 근접/희귀 자산 성공 데이터는 삭제하지 않았다. 마지막 동적 child는 실제 움직임 QA를 통과했다.

최종 root의 `selected_ledger.jsonl`, `replacement_mapping.json`, `adopted_manifest.json`,
`coverage_final.json`, `validation_report.json`, `live_replacement_evidence.json`,
`resume_validation.json`, `cpu_infos_final`을 사용한다. 기존 manifest/runtime는 수정하지 않았으며,
최신 ledger에 이전 ledger를 덧붙이지 않는다. 이전 v139 4,974/26과 v137 4,975/25는 진행 이력이다.
대시보드 8075는 v140의 **5,000/0/complete**를 표시하고 suite Sunny13도 5,000으로 반영한다.
대시보드의 원래 manifest는 패밀리 목표·환경 대응용이며 실제 대체 자산 coverage는 adopted 명세로 검증한다.
collector는 종료했고 MR을 꺼도 된다. 이 output에서 resume/run은 재촬영하지 않는다.

가시성 좋은 대체를 채택하는 것은 부족 0.52%의 수집 전략이며
평가 모델의 성능을 보고 test/val을 선별하는 절차가 아니다. 근접의 어려운 정상 승인 데이터는 유지한다.

5K 완료는 실주행 모델 정확도 인증이 아니다. 남은 5 환경의 새 배치/연속 Ego 다양화·총 30K 계획,
crop/투영/runtime 최적화 및 rosbag dev-val/독립 test 계약은 유지한다.
