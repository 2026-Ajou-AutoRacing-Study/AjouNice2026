# Base5 Sunny 13시 기본값·이력 정정

2026-10-07 / EVIDENCE·운영 정정. 사용자가 현재 MR 기본값과 실제 수집이
Sunny13시라고 확인했다. 최근 문서와 생성기 설정의 Sunny11시는 잘못된 표기였다.
이 정정은 MR 시간을 변경하거나 새 촬영을 수행한 결과가 아니다.

## 확인한 근거

- 과거 Base3 정적/동적 보강 config의 `environment.time`은 `13:00`이다.
  `static_classcentric_base3_batch008_dynamic_ped.json`과 관련 pilot/replacement,
  `base3_static_batch008_dynamic_ped.json` 및 `dynamic_object_capture_v1.json`을 확인했다.
- 과거 Sunny13 production 명령/캡처 문서도 MR에서13시를 수동 설정하도록 안내했다.
- 최근 Base5 config는`collection_environment.hour=11`로 기록되어 있었고
  v105 manifest에 그 선언값이 복사됐다. 이는 자동 적용/실측 근거가 아니다.
- Base5 metadata에는 캡처 시각의 환경 측정값이 없고, 시나리오에도 시간/날씨 설정이
  없다. ScenarioLoad로 Sunny11시가 자동 적용됐다고 해석하면 안 된다.
- 실제 현재 환경의 근거는 사용자 확인이다. 개별 과거 촬영의 시간을 센서/패킷에서
  독립 검증했다고 주장하지 않는다. 대회6환경(11/13/15×Sunny/Foggy) 계획은 유지한다.

## 수정한 기본값과 문서

- 새 명시적 config:`configs/base5_independent_5k_sunny13.json`.
  dataset ID도`base5_independent_5k_sunny13_261007`,weather sunny/hour13이다.
- 기존 기본 경로`base5_independent_5k_sunny11.json`은 **파일명만 호환 alias**로 유지한다.
  내용은 위 config와 동일한Sunny13이다. 기본 생성기로 실행해도13시로 선언된다.
  generator 코드의 기본 경로를 바꾸지 않은 이유는 기존 campaign의 code hash/resume를
  깨뜨리지 않기 위해서다. 두 config의 동일성은 단위검사로 확인한다.
- CURRENT/AGENTS/ACTIVE/README/계획/런북/차량 근접 근거/누적 Base5 이력의
  기준 환경 표기를13시로 정정했다. 과거11시 환경이 실제 존재했다는 주장으로 남기지 않는다.
- 이 변경으로 geometry/거리/quota/split/GT/평가 수치/캡처 비용을 바꾸지 않았다.

## 기존 산출물 보존과 새 manifest

원본 capture·metadata·scenario·manifest·ledger·resume contract는 덮어쓰지 않는다.
각`~/morai_cap/base5_v2_261006`/`base5_v2_261007`의
`collection_environment_correction.json`에 기본 환경 정정과 근거를 따로 남겼다.
오래된 경로·파일명 또는 원본 header의11은 당시 잘못 기록된 선언값으로 읽는다.

새 수집용 header를 정정한 별도 manifest:

```text
~/morai_cap/base5_v2_261007/final_5k_ordered_sunny13_v109/manifest.json
```

원본 ordered v105의5000scene row·scenario hash·metadata 경로·quota·순서·split은
모두 동일하다. dataset/environment header와 그 config provenance만 정정했다.
같은 폴더의`environment_correction.json`에 원본/정정 manifest hash를 기록했다.
재배치·재캡처를 한 것이 아니다.

- 새 config SHA256:`d58e15791a32005655da1e343e109a718646b0859bcd4fdb2918d85e03d35948`.
- 새 manifest SHA256:`1b8a6ced7d2af06631e1cbd2897a4464adc897ef833023f6f4f373e5c0d04523`.
- 기존 원본 manifest SHA는 그대로다. 이전 evidence의 checksum은 원본 파일을 지칭한다.

**이미 진행/재개 중인 수집은 기존 명령·manifest·output을 유지한다.**
중간에 정정 manifest로 갈아끼우면 의도대로 resume contract mismatch가 난다.
새 manifest는 새 output에서만 사용하고 기존 촬영은 정정 기록과 함께13시 기본 이력으로
관리한다. 환경을 실제로 바꾼11/15시·Foggy 수집은 별도 campaign이다.

정정 helper는`correct_base5_collection_environment.py`이며 사용자 환경 확인을
`--operator-confirmed`로 명시해야만 새 파일을 생성한다. overwrite/in-place 정정은 거부한다.

## 정정 후 검증

- Base5 단위 검사 241개 PASS. 기본 alias와 명시적 config가 모두 Sunny 13시이며 동일함을 확인했다.
- 새 5K manifest 전수 OFFLINE_PASS, 5000 unique scene, 오류 0. 기존 5000 scene row는 그대로다.
- 기존 v108 완료 campaign의 원본 manifest로 resume 검사를 통과했다. 추가 촬영은 수행하지 않았다.
- 검사 대상 기존 sensor 파일 610개의 내용·mtime·크기는 변하지 않았다.
- 이는 환경 표기 정정 검증이며, 센서 영상으로 개별 과거 프레임의 시간을 실측한 결과는 아니다.
