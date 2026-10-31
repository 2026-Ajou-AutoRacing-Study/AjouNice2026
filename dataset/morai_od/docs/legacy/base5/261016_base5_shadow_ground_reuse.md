# 음영 이동 경로의 기존 지면 준비 재사용

문서 상태: EVIDENCE. 작성일: 2026-10-16. 현재 운영 상태는 [Base5 CURRENT](BASE5_V2_CURRENT.md)를 우선한다.

## 목적과 범위

기존 지면 측정 결과를 명시적으로 등록한 뒤 같은 장면의 새 캡처에 재사용한다.
과거 캡처를 새 학습 데이터로 합산하거나, 예전 수집 output에 변경된 코드로 강제 resume하는 기능이 아니다.
등록하지 않은 장면의 기존 수집·지면 준비 경로는 유지한다. 전체 음영 이동 24장을 미리 재촬영하지 않는다.

이전 수집기는 현재 실행 output 안의 준비 deck만 재사용했다. 다른 실행에 있는 준비 결과를 전역 검색해 자동 적용하지 않았다.
이번 구현도 이름이나 인접 위치로 추정하지 않는다. source ID, 재사용 방식, 준비 manifest, 과거 selected ledger를 직접 지정해야 한다.

## 실제로 확인된 준비 자료

`candidate_v153d_fast`와 후속 `candidate_v154c_spread`·`candidate_v154d_spread`의 음영 이동 source는 아래 네 장면에 대해 같은 대조 결과를 보였다.
이는 전체 24장 준비가 완료됐다는 뜻이 아니다.

| Source ID 끝자리 | 자산 | 등록 방식 | 과거 calibration 캡처 | 지면 proof 파일 |
|---|---|---|---:|---:|
| 00000 | Dog1 | `exact_source` | 2 | 6 |
| 00449 | Dog2 | `exact_source` | 3 | 9 |
| 00062 | Ped_Pet1 | `same_quota_replacement` | 5 | 15 |
| 00140 | Woman2 | `same_quota_replacement` | 5 | 15 |

앞의 두 장면은 준비 직전 scenario SHA와 metadata SHA가 현재 source와 모두 일치한다.
뒤의 두 장면은 자산·spec·동작·split·group은 같지만 과거 복구 과정에서 geometry가 바뀌었다.
따라서 별도의 명시적 대체 선택 없이는 적용하지 않는다. 이번 소량 실증 대상으로는 앞의 두 장면만 사용한다.

근거 prefix는 다음 경로다.

```text
artifacts/evaluation/base5_obstacle_range_261015/qualification_animated8_v152b/
```

이 prefix의 `selected_ledger.jsonl`에 네 준비 장면의 실제 승인 캡처가 기록돼 있다.
exact 두 장면의 준비 manifest는 각각 다음 경로다.

```text
chunk_0000/recovery/sunny13_sunny11_b5v2_00000/sunny13_sunny11_b5v2_00000/round_0/sunny13_sunny11_b5v2_00000/motion_ground_deck/manifest.json
chunk_0000/recovery/sunny13_sunny11_b5v2_00449/sunny13_sunny11_b5v2_00449/round_0/sunny13_sunny11_b5v2_00449/motion_ground_deck/manifest.json
```

선택적 대체 두 장면은 같은 prefix의 `chunk_0001/recovery/<source ID>/<source ID>/round_1/<source ID>_replace/motion_ground_deck/manifest.json`이다.
기존 v140의 준비 24장은 현재 source와 seed·실제 경로가 달라 직접 재사용 대상으로 등록하지 않았다.

## 등록과 실행 계약

새 `prepare_base5_shadow_ground_reuse.py`가 fresh output에 `registry.json`과 파생 `manifest.json`을 만든다.
원본 full manifest를 한 번 pin하고, 파생 manifest에는 registry 경로와 SHA만 config에 추가한다.
기존 scene 배열·ID·spec·seed·자산 할당은 그대로다. 따라서 subset이나 manifest 사본을 만들 때 순환 해시가 필요 없다.

등록 시 다음 근거를 대조하고 pin한다.

- 현재 source scenario·metadata와 준비 직전 manifest, 준비 완료 scenario·metadata.
- 환경 태그, 자산 UID/data ID/이름/container, 실제·요청 motion, population pool, split·group 계보.
- 현재 sensor JSON과 과거 raw의 sensor reference·session sensor SHA.
- 각 지면 proof의 실제 파일 SHA, waypoint와 metadata의 좌표, asset별 pivot offset.
- 과거 calibration ledger·session contract·Park, 선택을 명시한 selected ledger와 승인 raw의 provenance.
- 각 역사 session contract가 기록한 child manifest SHA와 해당 row의 scenario SHA·입력 metadata. 실제 sibling launch deck가 없으면 거부한다.
- CollisionData 관측, 무접촉 summary와 audit의 일치. 무수신 또는 접촉 이력이 있으면 거부한다.

역사 ledger는 읽기만 한다. 불완전한 마지막 줄도 복구·개행 추가하지 않고 거부한다.
등록 후 sensor·source·terrain·selected ledger·session 근거가 바뀌면 `ShadowReuseContractError(RuntimeError)`로 중단한다.
ordinary 장면 부족분으로 변환하지 않는다.

품질 루프는 등록된 원래 ID에 대해서만 새 `motion_reuse_deck`를 만든다.
이때 과거 terrain/static calibration을 다시 촬영하지 않고, 지면이 준비된 scenario를 현재 수집기로 새로 캡처한다.
가시성·지면·실제 이동·부유·충돌·Ego 정지·종료 Park 검사는 그대로다.
아직 검증하지 않은 다른 경로로 지면을 외삽하지 않는다. 같은 quota의 새 복구 geometry에는 자동 적용하지 않는다.

현재 실행을 resume할 때도 registry와 모든 외부 근거를 다시 확인한다.
이미 만들어진 adoption deck은 덮어쓰지 않으며, 완료 raw의 metadata가 계획 입력과 달라졌는지도 검사한다.

등록 CLI 형태는 다음과 같다. `<...>`는 동결된 실제 경로로 치환한다.

```bash
python3 dataset/morai_od/scripts/prepare_base5_shadow_ground_reuse.py \
  --manifest <현재-full-source>/manifest.json \
  --output <새-registry-output> \
  --selected-ledger <과거-qualification>/selected_ledger.jsonl \
  --prepared exact_source:sunny13_sunny11_b5v2_00000:<00000의-motion_ground_deck>/manifest.json \
  --prepared exact_source:sunny13_sunny11_b5v2_00449:<00449의-motion_ground_deck>/manifest.json
```

## 검증과 한계

신규 helper 테스트는 실제 독립 지면 proof 형식을 사용하는 CPU fixture로 등록·명시적 대체·불변성·오염 거부·subset·완료 resume를 검사한다.
신규 58개와 기존 quality loop·motion ground·campaign·safety 회귀 56개, 합계 114개가 통과했다.
품질 루프의 가짜 child 시험에서는 calibration 호출 없이 `initial` native 캡처 한 번만 호출되는 것을 확인한다.
이후 [v154 최종 검토](261016_base5_precollection_final_review.md)에서 exact 두 장면을 실제 simulator로 새로 캡처했다.
기존 calibration 5회를 반복하지 않고 새 native 두 장 모두 승인됐으며, 완료 replay의 추가 캡처는 0회였다.
부분·완료 raw 보존과 현재 source/sensor pin도 확인했다. 이 결과를 다른 22개 경로의 준비 완료로 일반화하지 않는다.

기존 지면은 LiDAR 국소 평면과 정적 annotation을 이용한 근거다. 연속 mesh 접촉이나 완벽한 센서 시간 동기화를 인증하지 않는다.
복합 자산의 parent/component 라벨 모호성과 새 이동 장면의 실제 동작은 각각 별도로 확인해야 한다.
이 재사용 기능은 동기 속도 GT나 새로운 모델 AP 개선을 주장하지 않는다.
