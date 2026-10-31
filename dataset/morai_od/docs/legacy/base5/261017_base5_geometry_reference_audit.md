# Base5 기존 5K native 치수 전수 감사와 크기 reference 분리

문서 상태: EVIDENCE. 기준일: 2026-10-17, Asia/Seoul.
기존 완료 Sunny13 5K의 selected ledger 전체를 CPU로 읽고 원본 annotation을 검산했다.
MR·ROS·GPU·본수집·학습은 실행하지 않았다. 기존 크기 표·raw·GT·v155m release는 수정하지 않았다.

## 네 가지 크기 계약

| 자료 | 의미와 소비 범위 | 이번 처리 |
|---|---|---|
| 원형 unit reference | 자산 로컬 native 치수, 확대 진단의 원형 비교값 | 안정 관측 18종의 새 진단용 파생 reference 생성 |
| Conservative placement envelope | spacing·footprint·초기 배치에 쓰는 여유를 포함한 추정치 | 기존 값 유지. unit 중앙값으로 일괄 축소하지 않음 |
| Pivot·height initializer | 저장 pivot·MGeo·calibrated offset으로 정한 배치 초기화 근사 | 기존 원문과 proxy 의미 보존. mesh 접촉 정답으로 승격하지 않음 |
| 프레임별 native GT | 해당 캡처의 중심·full RPY·치수 및 승인된 모서리 정책, 파생 upright 학습 박스 | 원본과 기존 파생 GT 모두 불변 |

과거 배치용 추정치와 현재 native 원형이 다르다는 이유로 기존 scale 1 학습 GT가 틀렸다고
결론 내리지 않는다. 거리 분류용 yaw 제거 upright 중앙값도 native 로컬 원형 치수와 다른 통계량이다.
구 [거리 근거](261015_base5_primary_obstacle_range.md#거리-분류에는-배치용-envelope가-아닌-native-반복-치수를-사용한다)와
[v155 실측](261016_base5_native_scale_and_extent_evidence.md#자산별-규격과-배치-초기화)은 당시 근거로 보존한다.

## 조사 모수와 검산

입력은 canonical `collection_5k_sunny13_flexible_v140/selected_ledger.jsonl` 하나다.
부모의 성공 ledger를 다시 연결하거나 최종 학습 GT에 남은 성분만 추리는 방식이 아니다.
selected ledger SHA-256은 기존
`026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43`과 같다.

| 항목 | 결과 |
|---|---:|
| Selected 장면 | 5,000장, train 4,255 / val 745 |
| 전체 native 성분 | 11,372개 |
| 그중 복합 부모의 native 성분 | 1,134개. 단일 원형 추정에 혼합하지 않음 |
| Catalog semantic obstacle의 물리 19종 | 3,299개 성분 / 1,637장 |
| 물리 19종의 정적·scale 1·단일 부모 성분 | 3,299개 |
| 물리 19종의 nonunit / 이동 unit / 복합 성분 | 각각 0 / 0 / 0개 |
| 과거 거리 reference의 upright q50·관측 수 재현 | 19/19종 일치 |
| 물리 19종의 summary와 native 모서리 치수 불일치 | 0개 |
| 소비한 입력 파일의 시작/종료 SHA 불변 | 18,313개 |

매 장면의 saved audit가 selected ledger의 audit와 같고, scenario SHA와 catalog·runtime·native 성분의
이름/DataID/UID/container가 맞는지 검사한다. 정적·scale 1·단일 부모를 개별 분류하고, 물리 성분이
있는 장면에서는 정확히 같은 timestamp의 Instance와 Instance_8Points를 연결한다. native full RPY의
로컬 프레임에서 모서리를 검산하며 중심·회전·대칭·8개 꼭짓점 조합과 원문 SHA를 대조한다.
분석을 위해 모서리 치수 변화 검사를 허용한 것이지 GT를 다시 채택하거나 scenario scale을 곱한 것이 아니다.

## 과거 통계·현재 reference·관측치의 차이

다음 값은 X/Y/Z 또는 native 로컬 L/W/H의 m 단위 값이다. 반올림은 문서 표시에만 적용했다.
`report.json`에는 19종 각각의 원문 값·q10/q50/q90·최소/최대·크기 상태·출처·차이/비율이 있다.

| 자산 | 기존 배치 envelope | 과거 거리용 upright q50 | 기존 별도 실측 unit reference | 이번 원형 모서리 local q50 |
|---|---|---|---|---|
| Lane_Separation_Bar | .292/.283/1.338 | .213/.213/.752 | 없음 | .210/.210/.750 |
| Bollard01 | .457/.440/1.596 | .369/.370/.856 | 없음 | .366/.366/.853 |
| PE_Firewall_Orange | .698/1.579/1.046 | .604/1.361/.907 | .600/1.358/.900 | .600/1.358/.900 |
| Steel_Barricade | 1.155/3.219/1.095 | 1.086/3.018/1.045 | 1.081/3.013/1.025 | 1.081/3.013/1.025 |
| Concrete_Barricade01 | .442/2.172/.920 | .393/1.921/.821 | .390/1.918/.812 | .390/1.918/.812 |

Upright q50는 각 관측의 roll/pitch에 따라 둘레가 커질 수 있다. 원형 local q50는 RPY 프레임으로
되돌린 실제 모서리 범위다. 따라서 위 숫자의 차이를 곧바로 오차나 GT 교체 사유로 쓰지 않는다.
기존 envelope는 generator와 같은 reference 우선순위를 따라 읽었지만 어느 원본도 수정하지 않았다.

정적 unit 관측의 모서리 치수를 2mm/0.5%의 분석용 조건으로 나누면 기존 5K의 19종은 각각 한 상태다.
이 조건은 QA 허용 오차가 아니며, 입력이 현재 시뮬레이터의 모든 자산 상태를 대표한다는 뜻도 아니다.
모서리 local 축과 Unity scale 축은 별개다. 기존 승인 정책의 축 대응은 출처와 함께 표시하고,
원형 관측만으로 나머지 자산의 Unity scale 축을 새로 인증하지 않는다.

## RedBarrel의 독립 변동은 별도 보존

기존 5K의 RedBarrel 164개는 한 크기 상태지만, 그것만으로 현재 자산 안정성을 승인하지 않는다.
`redbarrel_native_variation_v155l.json`의 별도 4회 캡처·12개 관측을 해당 원문 24개 annotation/모서리와
metadata/scenario에서 다시 계산했다. 정적 unit 6개와 nonunit 6개를 나눴고 기존 5K 분포에 합산하지 않았다.

| 독립 unit 상태 | 관측 수 | 모서리 local q50 L/W/H |
|---|---:|---|
| 작은 상태 | 4 | .572698/.572403/.919410m |
| 큰 상태 | 2 | .715879/.715494/1.149262m |

약 1.25배 차이를 하나의 중앙값에 섞지 않는다. RedBarrel은 새 진단 unit reference에서도 제외한다.
원인·LOD·캐시의 인과관계는 입증하지 않았으며 기존 확대 보류를 해제하지 않는다.

## 새 산출물과 소비 경계

최종 artifact root는 `artifacts/evaluation/base5_geometry_reference_261017/full5k_v3/`다.
앞선 `full5k_v1`·`full5k_v2`는 개발 중 파생 결과이며 현재 소비 대상으로 사용하지 않는다.

| 파일 | 내용 | SHA-256 |
|---|---|---|
| `report.json` | 19종 비교·분류·독립 RedBarrel 재검산·네 계약 | `5ae2c02c4c9d635241043de8d8465146fbf3a5edca7e798ceb9a807e7dd918a7` |
| `observations.json` | canonical 물리 성분 3,299개, 개별 native 모서리·높이 초기화 근거 | `fcf9478d119ea4f18d2f574021a900f7d9f9b811759276ad8aeaadea16288ceb` |
| `source_pins.json` | 실제 소비 입력 18,313개의 SHA/bytes | `ce9a40557f79fd45f9c28536cf599947770a78b888c639060147af0eef589544` |
| `diagnostic_unit_reference.json` | 안정 관측 18종의 진단용 unit reference, RedBarrel 제외 | `81913d36079a6b157fc8affcdc8ee23713643b0dd61ce4440c4d6dc108602559` |

새 reference는 `production_approved=false`다. `bbox_size_m`은 기존 raw의 native local 원형 치수지만,
`pivot_to_bbox_center_local_m`은 **기존 envelope의 calibrated pivot proxy를 축 비율로 재표현한 근사**다.
새로운 pivot·접지 실측이 아니며 conservative envelope를 덮어쓰는 자료가 아니다.
진단 probe 생성기의 명시적인 `--unit-reference` 입력으로만 전달하고 파일 SHA를 동결한다.
해당 자산의 eligibility가 허용된 경우에만 사용하며, 새 확대/축/pivot/지면/외형의 실제 native 검증은 별도로 필요하다.
기존 native GT·가시성 기준·component 관계를 바꾸거나 확대 QA를 우회하는 입력으로 사용하지 않는다.

재현 명령은 새 output 디렉터리를 요구한다.

```bash
python3 dataset/morai_od/scripts/audit_base5_geometry_reference.py \
  --output artifacts/evaluation/base5_geometry_reference_261017/reproduction_new
```

새 검사 11개와 관련 회귀(native corners 13·unit reference 11·upright 5·과거 range audit 8),
총 48개가 통과했다. `git diff --check`도 통과했다. 이 결과는 수집 준비용 reference 감사이며
새 5K 완료·새 scale 승인·학습 AP·추론/추적/planner 성능 개선의 인증이 아니다.
