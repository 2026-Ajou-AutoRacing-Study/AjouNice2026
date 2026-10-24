# Base5 v42 — 32셀 비용·8셀 pause/resume·동적 준비시간 비교

2026-10-06. EVIDENCE. 본10K 캡처·학습·production 변경 없음.

## 대표32셀 새 경로 검증

guarded bridge + shared ROS chunk4 + 안전 review 재사용으로 한 campaign에서32/32 통과했다.
v41의 여러campaign 합산32셀과 달리 이번은 새 단일campaign의 결과다.
36회 실제촬영에는 terrain/검토/보완도 포함하며 최종quota는32다.
모든 boundary/finalPark 확인. EventCmd86회 중 짧은 handler종료 오류3회는 기존retry로
회복했고 timeout/미종료worker0이다. 모든 오류가 사라졌거나 장시간 인증됐다는 뜻은 아니다.

| 같은 대표 원본32셀 | v36 이력 | v42 새 경로 |
| --- | ---: | ---: |
| 전체 wall | 314.657초 | 233.02초 (driver232.917초) |
| 실제촬영/최종quota | 36/32 | 36/32 |
| ledger scene duration 합 | 228.419초 | 204.787초 |
| 나머지 wall | 86.239초 | 28.131초 |

전체 약25.98% 단축. 중복작업 제거·조건부 preflight·process/ROS 공유·서비스 guard가
함께 바뀐 순차 비교다. 단일 변경의 인과 효과나 반복 A/B 신뢰구간을 의미하지 않는다.
나머지wall에는 startup/preflight/Park/부모검증 등이 포함되며 순수Python 비용이 아니다.
원본scene은 대응하지만 동적 자세·보완payload는 매번 실제관측으로 달라질 수 있다.

새 비용report는 실제 `ledger.jsonl`의 session+id를 세어 seed reference와 resume 중복을
새 촬영으로 계산하지 않는다. 같은 계산으로 재집계한 family 가중10K 외삽은
**27.24→20.19시간**이다. 기존v36 report27.53과 계산방법이 달라 재집계치를 직접 비교한다.
coverage 우선32셀의 작은표본이며 전체pool·장시간저하·재시도율을 보장하지 않는다.
현재도 하루규모 수집 가능성을 고려해야 하며 하룻밤 완료를 약속하지 않는다.

32셀 최종 CPU변환은train26/val6/test0, GT48개:
vehicle21/pedestrian10/obstacle13/animal4. review/failed 편입0이다.
정책은lidar1_camera0, ROI[-100,-30,-3,100,30,2], velocity target 없음이다.

## chunk8 pause/resume

별도16셀pilot을 첫8셀 완료시 안전pause한 뒤 같은계약으로resume했다.
16/16통과, EventCmd37회 오류/timeout/미종료worker0. 두 실행wall46.33+56.09초다.
중간stop-after-chunks는 안전경계 pause이며 활성capture 중 강제Ctrl-C의 live시험은 아니다.
chunk4보다chunk8이 빠르다고 주장하는 대응 A/B는 하지 않았다.
전체 기본chunk4는 유지하며 chunk8은 검증된 opt-in 후보다.

## 동적 출발 여유9초→6초 후보

`prepare_base5_setup_budget_pilot.py`는 최대8셀만 준비한다. 원본spec/retry·Ego·split·
객체종류·목표 geometric center/size·자연속도를 고정하여 생성기로 다시 구성한다.
목표뒤쪽 출발거리만 줄이며 quota/속도/도착gate를 바꾸지 않는다. 재생성 중 다른
대상/geometry로 바뀌면 거부하고 임의resample하지 않는다. canonical10K 기본9초는 그대로다.

worker_m_1 걷기, Woman2 달리기, Ped_Pet1 두actor, Deer1+Dog2 두actor의4셀 비교:

| 조건 | 9초여유 | 6초여유 후보 |
| --- | ---: | ---: |
| wall | 59.39초 | 44.58초 |
| 최종quota/실제촬영 | 4/5 | 4/5 |
| EventCmd | 12회·짧은오류1회retry회복 | 11회·오류0 |
| timeout/미종료worker | 0/0 | 0/0 |

약24.94% 단축이다. 주된차이는 도착대기약3초×실제5촬영 감소이며 서비스오류 여부도
다르므로 전부를 budget 효과로만 단정하지 않는다. 속도를 늦추거나 정적pose로 바꾸지 않았다.
양쪽Z보완1셀을 독립재촬영했고 모든 최종충돌/지면/Z/도착/가시성 gate를 통과했다.
6초후보 CPU4장/GT8개(pedestrian4/animal4), train3/val1/test0 PASS.
모든human/animal 및 늦은service 준비 조건에서 성공한다고 확대하지 않는다.

## 완료resume snapshot 경합 수정

32셀 완료resume와 CPU변환을 동시에 실행하자 변환기가중간16행 aggregate를읽었다.
완료resume가검증하면서canonical ledger를부분목록으로다시발행했던것이 원인이다.
원본32행/센서는보존됐고 resume완료후 다시변환해32장/GT48개를확인했다.
부분변환출력 `batch_32_seed_guard_v42_infos`는 이력이며 학습용이 아니다.
올바른출력은 **`batch_32_seed_guard_v42_infos_final`** 이다.

완료resume는중간running snapshot을발행하지않고, 모든검증후 최종목록을atomic교체한다.
검증실패시 이전ledger를보존하고 status를halted로남긴다. 완료증거가누락되면
새촬영을시작하지않는다. 완료wall도검증시간으로덮어쓰지않도록수정했다.
동시읽기·추가촬영금지·원래wall보존 회귀테스트 포함Base5총132tests PASS.
wall보존 추가전의live완료resume에서122회동시읽기가모두4행, ledger SHA불변,
EventCmd12→12였다. 마지막wall보존변경후 새2셀11.43초/2quota를촬영했다.
완료resume 동안25회동시읽기 모두2행, ledger SHA불변, EventCmd5→5,
elapsed11.32331251190044초가검증후에도정확히보존됐다.

## 근거·재현

공통 root `/home/luv/morai_cap/base5_v2_261006/`:

- `batch_32_seed_guard_v42`, `batch_32_seed_guard_v42_cost.json`
- `cost_baseline_v36_recount_v42.json`
- `batch_16_pause8_v42`
- `setup_budget6_pilot_v42/manifest.json`, `offline_audit.json`: OFFLINE_PASS4셀
- `setup_budget9_capture_v42`, `setup_budget9_cost_v42.json`
- `setup_budget6_capture_v42`, `setup_budget6_cost_v42.json`, `setup_budget6_infos_v42`
- `batch_32_seed_guard_v42_infos_final`: 최종32장 변환
- `snapshot_fix_live2_v42`: 최종snapshot/wall보존 live

```bash
python3 dataset/morai_od/scripts/report_base5_batch_cost.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/batch_32_seed_guard_v42" \
  --manifest "$HOME/morai_cap/base5_v2_261006/capacity_pilot_v36/manifest.json" \
  --elapsed-sec 232.91738347604405 \
  --output /tmp/base5_batch_cost.json
```

구driver의 완료resume는elapsed를덮어쓰므로위cost원본과표의최초wall을사용한다.
위명령은기록된 최초driverwall을명시한다. 임의예상시간을넣어추정치를좋게만들지않는다.
새driver hash변경전campaign을현코드로강제resume하지않는다.

## 다음 판단

6초후보는별도deck으로검증했을뿐 canonical10K/분포/threshold는미변경이다.
다음은더다양한동적자산·준비지연조건에서6초후보를검증하고, 도착실패시동일quota를
9초여유로보충하는안전fallback을설계하는것이다. 충분한근거후새10Kdeck에반영한다.
정적cell의service/pose준비도남은주요비용이다. network재로드생략이나Park완화는금지한다.
본10K 캡처·학습은아직시작하지않는다.
