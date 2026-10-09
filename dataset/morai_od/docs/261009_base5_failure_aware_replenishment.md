# Base5 완료 부족분 반복 원인과 새 보충 경로

2026-10-10 최종: 사용자 종료4,948장 이후 자동 보충·실제 이동 계약 감사를 마쳤다.
**v139 최종4,974승인/26부족**, [최종 결과](261010_base5_adaptive_completion.md)가 최신 기준이다.
v137의4,975는 이동→정지 대체1셀 제외 전 집계다. collector 종료/MR 종료가능, 8075는v139다.
아래4,941/59 및8셀 수동 명령은 초기 검증 당시의 이력이며 지금 재실행하지 않는다.

2026-10-09 Sunny13 입력 v132, 캡처 runtime v133을 유지한다. 원래5K 셀을 모두 처리한 결과는
4,930승인/70부족이다. 기존 보충에서15셀 중3장을 추가한 뒤 안전하게 일시정지했다.
원래4,933승인과 모든 성공·실패 raw는 보존하며 현재 보충은 별도 v135 output을 사용한다.
v134b에서3장을 복구했지만 혼합 후보의 객체 수 변경을 사전 계약 검사에서 잡아 중단했다.
v135는 전체 spec이 다른 후보를 생성 단계에서 제외하며, v134b의 완료13시도/복구3장을
실제 raw와 기존 캡처 계약으로 검증해 명시적으로 import한다. halted journal을 수정하거나
hash를 강제 갱신하지 않는다. 기존 실패 시도도 가져와 누적 상한을 유지한다.

## 반복 원인

바깥 보충은 새 geometry를 만들었다. 그러나 내부 quality loop의 재배치 retry와 다음 quota
회차의 initial이 같은 retry 번호에서 출발해 동일한 payload를 다시 캡처했다.
Concrete_Barricade01과 CargoBox에서 실제 scenario SHA256 일치를 확인했다.
바깥 후보가 새로 생성됐다는 사실만으로 실제 캡처에 중복이 없다고 판단할 수 없다.

절대 위치를 옮겨도 상대 시야가 같으면 실패한다. 측방 초근접 바리게이트는 LiDAR0점과
camera 근거0, 원거리 CargoBox는 작은 camera rectangle과LiDAR0점, 믹서 트럭은 정지 배치 후
실제 이동이 원인이었다. 이들은 동일한 해법으로 해결되는 실패가 아니다.

## 새 보충 정책

- 원본 input/runtime/journal은 수정하지 않는다. 원본 main과 기존 보충을 원래 validator로
  검증한 뒤 새 output의 선택 ledger에 성공 raw 참조를 합친다. 원본을 삭제하거나 옮기지 않는다.
- primary 자산, 전체 spec, split, scene group, 환경 config를 유지한다. 실패 payload와
  사실상 같은 상대 위치·yaw를 제외한다. 파일 이름만 바꾸는 것은 새 후보가 아니다.
- 기본16개의 유효한 후보를 최대512개 proposal 안에서 찾는다. 카메라 사영 크기,
  LiDAR 수직 beam 교차 proxy, 보조 객체의 각도 가림과 기존 실패 시야와의 차이로 순위를 정한다.
  가시성 인증이 아닌 후보 선정이며 실제 native annotation과 기존 QA가 승인을 결정한다.
- 선택한 정적 후보는 raw runner로 한 번만 찍는다. 셀당 최대2개 후보가 누적 상한이며
  resume 반복으로 초기화되지 않는다. 실패 raw를 보존하고 상한을 소진한 셀은 부족분으로 남긴다.
- 통신·최종Park·증거 무결성이 불명확하면 전체 중단한다. 확인된 collision/정지 scene 실패만
  기존 증거 검사 후 건너뛴다. GT/가시성/정지/Z 기준은 완화하지 않는다.
- raw 완료와 journal commit 사이의 crash는 기존 raw를 검증해 commit하며 다시 찍지 않는다.
- 초기 부족분64셀은 정적,3셀은 실제 이동 actor를 포함한다. 이동3셀은 별도 지면 준비가
  필요한 부족분으로 남긴다. 정적 one-shot으로 우회하거나 정지 모습으로 대체하지 않는다.

## 시간 제한과 지도 캐시

권장 entrypoint는 `run_base5_failure_aware_replenishment_fast.py`다. 기존 supplement driver와
캡처 코드는 그대로 사용하고 추가 scheduler hash를 별도로 검증한다. 공유 context에 MGeo
인덱스, 지역 polygon과 자산 높이 cache를 준비한다. 두 후보 A/B에서 scenario와 metadata가
완전히 같았고 후보당 생성은1.505→0.021초,1.542→0.006초였다. 전체 보충시간 감소율은 아니다.

기본15분, 아래 명령은10분 예산이다. 예산이 끝나면 현재 시도와Park를 마친 경계에서
PAUSE한다. 즉시 kill하는 hard deadline은 아니므로 안전 처리 시간이 추가될 수 있다.
사용자 PAUSE는 자동 제거하지 않는다. 명시적 resume은 이 scheduler가 만든 시간 예산
PAUSE만 제거한다. 원본 수집 root의 PAUSE는 계속 유지한다.

## 검증 상태

초기 실제 MR4셀 중2셀을 복구했다. Lane_Separation_Bar는 실제LiDAR1점으로,
Concrete_Barricade01은LiDAR0점이나 실제camera 근거로 승인됐다. 믹서 트럭은 배치 후
이동했고 Bollard04는 sensor 근거가 부족해 각각2시도 후 부족분으로 남았다.
모든 세션의 최종Park를 검증했고 원래4,933행은 내용 그대로 선택 ledger에 유지했다.
최종 소량 검증은15개 부족 셀/25개 실제 one-shot이며8개 셀을 복구했다.
현재 **paused/4,941승인/59부족**이다. 새 시도끼리 동일payload0건, 기존 보충의
실패payload 재촬영0건, 원본 ledger/quota/journal hash와4,933행 내용 불변이다.
Lane_Separation_Bar2셀,Concrete_Barricade01,YellowBarrel,CargoBox,Bollard01두 셀,
Santafe혼합 셀을 복구했다. CargoBox는 실제LiDAR2점, 일부 초근접은camera 근거다.
같은 제약 안에서 믹서/버스의 배치 후 이동, 일부 낮은 객체의 가시성 및 혼합 가시성
부족은 남았다. 실패를 성공으로 바꾸거나 전체5K를 다시 수집하지 않았다.

880.750초는 느린 초기 후보 준비와 보완 import를 포함한 보충 활성 누적시간이다.
cache를 적용한 최종8셀 묶음의 활성시간 증가는256.049초였다. 최초원본 검증 등
초기화 시간은 이 값에 모두 포함되지 않으며, 다른 셀의 완료 시간 보장은 아니다.
새 output의 `validation_report.json`에 동일payload/원본 불변/최종Park 결과를 남겼다.

실패 기반 검사19개와 scheduler 검사7개 PASS, 전체 Base5463개 PASS다.
완료 후 `--resume` 검증에서 추가 촬영0·25시도·4,941선택·활성시간 불변을 확인했다.
아직5K완료나 학습 충분성 인증이 아니다.

## 실행

master와 guarded bridge, MR Capture Mode/Sunny13을 유지한다. 원래
`run_base5_batch_campaign.py --resume --replenish`는 다시 실행하지 않는다.

```bash
ajou
BASE5_ROOT="$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release"

python3 -u dataset/morai_od/scripts/run_base5_failure_aware_replenishment_fast.py \
  --manifest "$BASE5_ROOT/environment_suite/sunny13/manifest.json" \
  --baseline "$BASE5_ROOT/collection_5k_final_v132_sunny13_recovered_v133" \
  --output "$BASE5_ROOT/collection_5k_sunny13_supplement_v135" \
  --import-supplement "$BASE5_ROOT/collection_5k_sunny13_supplement_v134b" \
  --resume --run \
  --stop-after-cells 8 \
  --time-budget-sec 600
```

추가 촬영 없이 검증만 하려면 `--run`을 제거한다. 기존 승인 행은 다시 찍지 않는다.
`--stop-after-cells 8`은 이번 실행에서 새로 처리하는 부족 셀 수이며 촬영 수가 아니다.
한 셀은 최대2후보를 찍을 수 있다. 누적 상한을 소진한 셀은 다시 실행하지 않는다.
새 canonical 선택은 새 output의 `selected_ledger.jsonl`이다. 원본과 새 ledger를 단순 연결하면
4,933장이 중복되므로 금지한다. 최초8075는 원래 paused 수집을 보고 있었으나 후속 수정에서
관측 root를 v135로 변경했다. 현재8075는 새 output의 승인/부족분과 보충 상태를 표시한다.
환경 suite는 같은 manifest hash의 현재 환경에 최신 선택을 반영한다. 기존 raw를 합산할 때는
아래 storage root를 명시하며 collector/input/journal은 수정하지 않는다.

```bash
ajou
BASE5_ROOT="$HOME/morai_cap/base5_v2_261008/final_appearance_5k_v132_release"
python3 -u dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$BASE5_ROOT/collection_5k_sunny13_supplement_v135" \
  --manifest "$BASE5_ROOT/environment_suite/sunny13/manifest.json" \
  --suite "$BASE5_ROOT/environment_suite/suite.json" \
  --storage-root "$BASE5_ROOT/collection_5k_final_v132_sunny13" \
  --storage-root "$BASE5_ROOT/collection_5k_final_v132_sunny13_recovered_v133" \
  --storage-root "$BASE5_ROOT/collection_5k_sunny13_supplement_v134b" \
  --port 8075
```

8075가 이미 실행 중이면 중복 실행하지 않는다. 재연결 시 대시보드만 재시작하고 수집기는 건드리지 않는다.
`--import-supplement`는 과거 성공·실패 완료 시도의 검증 참조다. resume에서도 같은 경로를
유지하며 v134b output의 state/ledger/raw를 삭제하거나 변경하지 않는다.

같은 제약에서 sensor 근거를 확보할 수 없는 셀은 별도 재설계 대상으로 남긴다.
5K를 맞추기 위해 거리 bin이나 미션 phase를 임의로 바꾸지 않는다.
