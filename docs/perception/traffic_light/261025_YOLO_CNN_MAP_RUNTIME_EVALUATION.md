# 신호등 YOLO·CNN·지도 연결 통합 검증

종류: EVIDENCE / 2026-10-25. 현재 운영값은 [CURRENT](CURRENT.md)다.
이 문서는 후보 구현·학습·검증 기록이며, production 승격을 뜻하지 않는다.
기존 YOLO 가중치·HSV·중앙 선택·정책 YAML·planning/control 기본값은 유지했다.

## 이번 작업의 결론

**새 YOLO + 지도 대상 선택 + 보강한 ResNet18 CNN v5를 다음 통합 시험 후보로 선택했다.**
같은 자연 현시 자료에서 기존 HSV의 색·좌화살표 혼동을 줄였으며, 특히 C의
가까운 신호등을 검출해 선택하는 것이 CNN 단독 교체보다 중요했다.
다만 정지 시험·조건부 native 일치율을 실주행 100%로 해석하지 않는다.
공식 황색 정책 확정과 대회 LAN·실제 planner 폐루프 검증은 남아 있다.

## 목표와 미션 범위

색/화살표를 잘 읽는 것, 올바른 신호등을 고르는 것, 판단에 전달할 신호로
해석하는 것은 서로 다른 문제다. 세 단계를 분리해서 비교한다.

| 단계 | 기존 운영 | 이번 후보 |
|---|---|---|
| housing 검출 | 기존 단일 클래스 YOLO, 640 | 6환경 YOLO26n 재학습, 960 |
| 미션 대상 선택 | 화면 중앙 기준 | route regulatory ID + 지도 housing의 이미지 투영 + 검출 박스 정합 |
| 원시 점등 관측 | HSV/형태 | ResNet18, R/Y/G/좌화살표 독립 4비트 |
| 현시 보정·시간 필터 | 현행 profile/smoother | 변경하지 않음, 보정 전후를 따로 진단 |
| planning 통과 판단 | 현행 BVP | 변경하지 않음 |

미션은 **A·B·C·D 네 구간**이다. C에는 물리 신호등 두 개가 있다.
eligible native ID는 A077/B091/C016+C018/D052이며, D는 옛 D-2다.
옛 D-1/201397은 미션 신호 판단에서 제외한다. 지도에 있는 모든 신호등을
학습할 수는 있어도, 모든 검출을 현재 미션의 신호로 승격하지 않는다.
지도에만 있는 다른 차량 신호등은 오연결을 막는 경쟁 후보로 사용한다.
지도 모드의 route helper는 다음 신호를 고를 때부터 네 미션 ID만 남긴다.
옛 D-1을 고른 뒤 매번0으로 버리는 구조에 머무르지 않는다. 기본 legacy 모드의
route mapping은 그대로다. 실제 v87 지도(1,302 lanelet)를 읽은 정적 검증에서
네 reference pose는 각각 A/B/C/D를 반환했다. C 뒤·옛 D-1 이전 위치에서는
기존 helper의201397 대신 후보가 다음 미션 D/201326을 반환함을 확인했다.
이는 지도·알고리즘 대조이며 실제 mission planner를 켠 폐루프 주행 인증은 아니다.

## 데이터·라벨

전체 근거는 `artifacts/evaluation/traffic_light_robustness_261025/`(이하 R)에 있다.
JPEG 원본은 재압축하지 않았고, native GT·과거 실험·기존 split도 덮어쓰지 않았다.

### YOLO

- 기존 직접 검토 사진 402장/636박스에 자연 신호 프로그램의 새 시점을 추가했다.
- 새 natural C/D 216장/342박스, A/B 198장/396박스.
- 최종 **816장/1,374박스**, train492/val120/development-test204.
- Sunny/Foggy × 11/13/15시를 포함한다. 같은 pose의 환경·상태·연속 프레임은
  같은 split이며, JPEG SHA가 split을 넘지 않는지 확인한다.
- 새 시점은 약 16/24/38/55m train, 29m val, 46m test에 횡위치·yaw 변화를 더했다.
  기존 개발 자료의 val/test도 섞여 있으므로 전부 새로운 독립 시험이라고 부르지 않는다.
- 화면 내 **식별 가능한 전면 수평 차량 신호등 housing, 원본 8×4px 이상**을
  연구 검출 클래스로 정의했다. 보행자/버스 전용·뒷면·측면·해상도 부족은 범위 밖이다.
  이 범위 밖까지 모든 신호등을 검출한다는 인증은 아니다.
- 기하 투영은 proposal일 뿐이다. 전신 이미지와 확대 crop을 직접 검토하고
  일부 투영 오차를 픽셀 기준으로 수정했다. A29의 추가 121 head는 실제 높이가
  약 3행이라 클래스 하한을 확정하지 못해 해당 18프레임 전체를 보류했다.
- 어두운 121/034가 앞면인지 모호했던 경우 exact native ID의 적색/녹색 변경으로
  같은 housing에서 램프 위치가 바뀌는 것을 확인했다. 이 인과 시험 사진 자체는
  학습에 추가하지 않았고, 비미션 head를 미션 allowlist로 넣지도 않았다.

주요 자료: `housing_detector_development_curated_v3.json`,
`housing_natural_CD_curated_v2.json`, `housing_natural_AB_curated_v1.json`,
`yolo_housing_combined_development_v1.json`.

### CNN v4: 첫 통합 후보

[앞선 crop 시험](261025_CNN_CROP_CANDIDATE_EVALUATION.md)의 546개는 보존했다.
실제 YOLO E28 박스를 native ID와 IoU≥0.5로 정확히 연결하고, node와 같은
정수 좌표 변환·사방 12px 여백을 적용한 246 crop을 train/val에 추가했다.
원래 승인 housing 전체 포함·다른 housing 무겹침·중복 crop 제외를 먼저 검사하고,
그 뒤 14개 contact sheet의 246개를 직접 시각 검토했다. 모델의 색 예측으로
라벨을 붙이지 않았다. test crop은 추가하지 않았다.

- 최종792개 = train432/val168/test192, native612 + 램프 없는 수동 배경180.
- 원본546개 row·라벨·split은 값 그대로 보존, 새246개만 새 QC 근거를 추가했다.
- 새 모델은 v3를 이어 학습한 것이 아니라 동일 ImageNet ResNet18에서 새로 학습했다.
- FP32/BN 통계 고정/30epoch/batch16/seed0, LR·증강·입력은 앞선 v3와 같다.
- val macro-F1, 동률이면 BCE로 E30 선택. val168/168, 기존 test192/192 조합 일치.
  기존 test는120신호+72배경이며 **새 YOLO 출력 전체나 새 주행의 정확도가 아니다**.
- `yolo_cnn_cropmix_v4/best.pt` SHA256:
  `2ed5437adc24fcc743cf2c63b35fb18506f3fecdd4916b71fcdf5581e2cabc47`.

주요 자료: `full_cnn_yolo_curated_v4.json`, `yolo_cnn_crop_qc_v2/`,
`yolo_cnn_cropmix_v4/{metrics.json,epochs.jsonl,predictions.jsonl}`.
QC는 assistant 직접 검토이며 독립 사람의 GT 인증은 아니다.

## 첫 YOLO 후보에서 발견한 score 문제

작은 개발 자료로 학습한 예비 YOLO의 E8은 mAP가 높았지만, val의 올바른 박스 score가
약0.024~0.102여서 운영 threshold0.25에서는 검출이 없었다. 같은 run의 E28은
동일 val에서 score 약0.913~0.979로 안정되어 threshold0.25에서90TP/0FP였다.
즉 **mAP가 높다는 것만으로 현재 threshold에서 사용할 수 있는 모델은 아니다**.
이것을 근거 없이 threshold만 크게 낮춰 배포하는 방식으로 해결하지 않았다.

최종 YOLO는100epoch/조기종료 OFF/save10으로 학습하고, val에서 고정 confidence0.25의
미션 head recall → housing F1 → FP → mAP50-95 순으로 checkpoint를 비교한다.
새 test 결과로 epoch나 threshold를 다시 선택하지 않는다. YOLO의 기본 NMS 경로와
새 one-to-one 출력 경로를 섞지 않는다. 예비 E28의 강제 one-to-one 시험은 오검출이
늘어 최종 후보에는 적용하지 않았다.

최종100epoch 학습은1,244.44초에 완료됐다. 저장 후보12개를960 val에서 비교해
E37에 해당하는 checkpoint를 선택했다. source `fit/weights/best.pt`와
복사한 `yolo_n960_final_v1/selected.pt`의 SHA256은 동일하다:
`848e67a257ed837c08e108cb5890bf98825681c074ce01f8e1e2f654a2313f5a`.

| 같은 고정 가중치·val120장/GT162 | TP | FP | FN |
|---|---:|---:|---:|
| 640 | 156 | 0 | 6 |
| **960, 최종 선택** | **162** | **0** | **0** |
| 1280 | 162 | 0 | 0 |

960의 TP score는 최소0.8829/중앙0.9252였다. 이 val에서는 기존 threshold0.25를
그대로 사용할 수 있었다. 1280의 mAP50-95는 더 높았지만 고정 threshold의
미검출·오검출을 줄이지 않아960을 유지했다. mAP 차이만으로 입력을 키우지 않는
선택 기준을 development-test 실행 전에 기록했다.
val에는5개 미션 head만 있고 다른 head GT는 없으므로 이 표를 비미션 신호의
오연결 안전성이나 전체 주행 성능으로 일반화하지 않는다.

`selection_frozen.json`과 `final_pipeline_freeze_v1.json`에 설정·SHA를 기록했다.
새 연속 자료를 보고 epoch·threshold·해상도를 다시 고르지 않는다.

### validation과 다른 development-test 결과

고정 후 실행한 204장/GT402의 IoU≥0.5 평가는 **TP329/FP81/FN73**였다.
미션 head만 보면 240/258이 맞았으며, 다른 전면 차량 신호등은 89/144였다.
따라서 val의 162/162를 전체 검출 성능으로 표현하면 안 된다.

- A077/B091/C016/C018은 각각36/36,36/36,54/54,54/54였다.
- D052는60/78이었다. 46m의18장은 실제 높은 score 박스가 있으나 약4px의
  수직 위치 차이로 IoU가 약0.44가 되어 strict 평가에서 누락됐다.
  기존12px crop 여백은 해당18장의 승인 housing 전체를 포함했다.
  이것이 그18장의 CNN 분류 성공을 증명하는 것은 아니다.
- FP81 중37개는 IoU0.25~0.5 경계,8개는 약한 겹침,36개는 GT와 무겹침이었다.
  실제 그림을 검토하면 일부는 후면형/버스 후보 패널이다. 중복 박스 pair는0개였고,
  모든 FP를 '없는 신호등 환각'으로 부르지 않는다. 지도 identity 검사가 별도로 필요하다.
- C011의 작은 인접 housing GT 외곽은 독립 재검토 여지가 있다. GT를 고치거나
  IoU 기준을 낮춰 최종 점수를 다시 유리하게 만들지 않았다.

원본 예측·12개 시각 진단은 `yolo_devtest_error_audit_v3/`에 보존했다.
IoU0.25의 별도 기하 진단은 strict 검출 평가나 실제 pipeline identity 성능을
대체하지 않는다. 이 결과로 모델·해상도·threshold를 재선정하지 않았다.

## 지도 연결: 운영 입력과 개발 정답의 구분

운영 후보 helper는 `/localization/kinematic_state`와 기존 TF를 읽는다.
이미지 시각의 `camera_front <- morai_map` 변환을 사용하고, 실제 카메라 extrinsic과
명시한 센서 config가 맞는지 대조한다. native API·Ego는 입력이 아니다.
이미 보정된 localization/TF에 −1.347°를 다시 적용하지 않는다.

개발 녹화의 Ego는 시나리오 위치·평가 정답·정지/P 확인에만 쓴다.
실ROS 배선 시험의 route oracle도 explicit pose를 미션 ID로 바꾸는 **개발 전용**이다.
이 시험을 mission planner route helper의 실제 폐루프 검증이라고 부르지 않는다.

실제 측위 covariance의 Zσ=1m가 수평 탐색 폭까지 부풀리는 축 결합을 찾아,
XY·Z·yaw의 역할을 나눴다. Z=1m는 실제 설정이므로 임의로 작은 값으로 낮추지 않았다.
기존 val로 수평90/수직180px 탐색 상한과 horizontal-housing 종횡비 guard를 검토했다.
기존 정상 val66/66 유지, 다른 head 음성18개 오연결0건이었다.
이18개가 모든 실전 오연결 확률을 보증하지는 않는다.
별도 독립 geometry 회귀에서 승인 박스를 검출 대신 넣었을 때816/816 미션 선택,
다른 head 박스만 넣는306건에서 오연결0건이었다. 실제 YOLO·측위·가림 성능이 아닌
명시한 covariance 조건의 선택기 회귀이며 `map_selector_independent_manual_box_challenge_v1.json`에 보존했다.

helper/node는 이미지 stamp·route·selector 계약 버전과 config를 맞춘다.
잘못된 대상·오래된 TF·깨진 패킷을 다른 신호등의 초록으로 대체하지 않는다.
동시에 missing을 숨기지 않고 원인·검출 수·선택 head를 진단한다.
분류기 실패도 묵시적 HSV 전환으로 숨기지 않는다. 기존 HSV를 쓰려면 명시적으로 선택한다.

## 연속 평가와 정책 경계

새 pose A/B/C/D × Sunny13/Foggy15에서 각130초 자연 주기를 녹화했다.
SceneLoad 없이 각 구간을 연속 관찰했고 **969 JPEG**, 오류0개를 확보했다.
JPEG마다 exact native ID의 전후 상태를 보존했다. 전후 현시가 바뀌는 프레임은
그 상태를 임의로 고르지 않고 reference unknown으로 둔다.

8개 전신 reference와51개 crop sheet의1,211개 named mission housing을 직접 검토했다.
이 자료는 대상 identity 평가용이며 **전체 화면 검출 학습 라벨이 아니다**.
native state 일치는 픽셀 가시성·독립 정답·규정 준수 인증과 구별한다.
동일 JPEG라도 stamp가 다른 연속 관측은 보존한다. sparse 사진에 운영7/4 smoother를
그대로 적용하거나10Hz로 복제하지 않는다. 전환 지연은 실제ROS 로그에서 따로 본다.

### 고정된 자연 현시 비교 결과

총 969장 중 native 전후 값이 바뀌지 않은 **같은 939장**을 비교했다.
전환 중인 30장은 unknown으로 따로 남겼으며 성공으로 합산하지 않았다.
각 비교의 eligible JPEG 집합이 정확히 같은지도 대조했다.

| detector / 선택 / 분류 | 원시 4비트 일치 / 939 | 좌화살표 TP / 116 | 정책 보정 후 일치 / 939 |
|---|---:|---:|---:|
| 기존 640 / 중앙 / HSV | 856 | 41 | 865 |
| 기존 640 / 중앙 / 새 CNN | 891 | 68 | 939 |
| 새 960 / 중앙 / 새 CNN | 891 | 68 | 939 |
| 새 960 / 지도 / HSV | 814 | 0 | 866 |
| **새 960 / 지도 / 새 CNN** | **939** | **116** | **939** |

핵심은 C다. 기존 YOLO는 작은 뒤쪽 C018만 검출했고, 새 YOLO는 가까운 C016도
검출했다. 중앙 선택은 새 박스가 있어도 뒤쪽 것을 계속 골랐다. 지도 선택이
더 큰 C016을 고르면서 CNN의 C 좌화살표 누락 48건이 해소됐다. 두 C 신호는
같은 프로그램이라 정책 보정으로 원시 누락이 가려질 수 있다는 점도 확인했다.

같은 자료의 named housing identity gate(IoU≥0.25)에서는 기존 YOLO 969/1,211,
새 YOLO 1,211/1,211이었다. 이 자료는 전체 화면 라벨이 아니므로 검출 AP나
전체 비미션 FP 평가로 표현하지 않는다. 지도 선택의 identity 오류는 이 969장에서는
0건이었으나, 가림·실주행·더 큰 측위 오차를 포괄한 인증은 아니다.

최종 지도+CNN의 환경별 조건부 일치는 Sunny13 469/469, Foggy15 470/470이었다.
네 구간별로는 A 237/237, B 234/234, C 236/236, D 232/232였다.
기존 방식도 이번 자료의 `false_go_proxy`는 0건이라 위 수치만으로 신호 위반율이
감소했다고 주장하지 않는다. 독립 사람이 붙인 lamp GT가 아닌 native 상태와의 일치다.

CNN은 12px 여백, 기존 HSV는 여백 0이므로 pure architecture 비교와는 다르다.
이를 확인하려고 저장된 동일 지도 박스에 HSV도 12px를 주어 CPU로만 재평가했다.
원래 HSV 결과를 939/939 그대로 재현했고, 원시 일치는 **HSV 0px 814개 →
HSV 12px 813개**, 두 경우 모두 좌화살표 0/116이었다. 같은 12px crop에서
CNN은 939개/좌화살표 116개였다. 이번 격차는 여백만으로 설명되지 않았다.
HSV threshold·정책·최종 후보를 이 결과에 맞춰 다시 고르지 않았다.

근거: `pipeline_baseline640_continuous969_gpu_v2.json`,
`pipeline_final960_continuous969_v1.json`, `hsv_crop_margin_ablation_final960_v1.json`.
실제 crop·기존/새 관측·native 상태를 나란히 보는 그림은
`pipeline_comparison_examples_v1/comparison_00.png`부터 3장에 있다.

각 이미지의 선택·분류·원시 관측·정책 보정·최종 출력과 단계별 callback wall time을
원본 stamp로 묶을 수 있도록 진단을 추가했다. wall time은 CUDA event latency가 아니며,
관측기 자체의 topic별 throttle/누락과 exact-join coverage도 별도 보고한다.
현 센서 C++ 수신기의 camera header는 **JPEG 첫 UDP 조각의 알고리즘 PC 수신 시각**이다.
따라서 header→출력 수치는 그 뒤의 조립·전달·처리 지연이지 MORAI 노출·렌더링·
Windows→Linux 물리 네트워크까지 포함한 photon-to-planner 지연 인증이 아니다.

황색·R+Y·unknown의 현재 인지/BVP 차이는
[정책 통합 대조](261025_SIGNAL_POLICY_INTEGRATION_AUDIT.md)를 본다.
현행 BVP의 `yellow_light_is_go`는 ‘이미 교차로에 진입했는가’를 확인하지 않고
amber를 GO로 처리한다. 이 코드 사실을 공식 허용으로 해석하지 않는다.
원시 분류 개선과 대회 규정 해석·통과 정책 확정은 별개의 작업이다.

## 실제 ROS 시험과 남은 UNKNOWN

실제 JPEG → localization/TF 투영 → YOLO → CNN → 표준 출력까지 별도 namespace로
확인했다. planner/controller를 켜 주행한 시험이 아니라 정지/P 조건의 입력·배선 시험이다.

첫 최종 모델 시험 `live_trial_monitor_v2/`는 8조건 중 마지막 Foggy15·D의 기록이
없었다. 카메라·Ego는 계속 들어왔지만 localization/TF/vision roslaunch가
06:04:11에 정상 종료(SIGINT)됐고 촬영기는 06:04:51에 끝났다. 따라서 모델 미검출이나
MORAI crash로 분류하지 않으며, 8조건 전체 성공으로 합산하지 않는다.
기존 자료의 7개 관측 구간에서 1,077개 selection 중1,072개가 연결됐다.
누락5개 중2개는 구간 끝이라 회복을 보지 못했으며 '모두 즉시 회복'으로 표현하지 않는다.

마지막 조건은 **촬영·정지/P 복귀 wait가 끝난 뒤에만** 시험 프로세스를 정리하는
단일 소유자 구조로 다시 실행했다. `live_trial_supervised_retest_v3/`에 보존했다.

- Foggy15·D, 기존 validation pose, 자연 현시45초/42 JPEG, 촬영 오류0개.
- 고정 capture window의 selection365/365가 C052에 연결됐고 missing은0개였다.
- classifier 관측367개 중317개 known, **50개 UNKNOWN**이었다. 모두
  `low_confidence_bits:left_arrow`이며 YOLO 미검출이나 다른 신호 선택 문제가 아니다.
  exact stamp로 runtime까지 연결된 UNKNOWN49개는 기존 profile의 빈 관측
  fallback으로 이어졌다. 여기서 `fail_open`이라는 이유 문자열은 기존 D 적색
  fallback이며, planning의 missing→GO와 같은 뜻이 아니다.
- UNKNOWN은 관측상33개 묶음, 최대8개 연속이었다. topic별 throttle이 있으므로
  이를 정확한 모든 처리 프레임의 지속 시간으로 단정하지 않는다.
- capture와 classifier를 exact stamp로 연결할 수 있는18장 중 native 전후가 안정된
  것은16장이었다. 그중 native33(R+L)9장은 UNKNOWN2개, **known이지만 L=0인7개**였다.
  이9장의 좌화살표 확률은 약0.21~0.43이었다. native1(R)6장과5(R+Y)1장은 known이었다.
  원본 SHA를 대조한11행 그림(위9장+R대조2장)에서도 native33에는 녹색 화살표형
  픽셀이 있고 R대조에는 없었다. 단순 GT 조회 오류나 UNKNOWN 빈도만의 문제가 아니다.
  `v3_D_left_pixel_audit_v1.png`, `v3_D_left_probability_audit_v1.json`에 보존했다.
  native GT는 여전히 독립 픽셀 GT 인증이 아니며 이 subset을 전체 정확도로 일반화하지 않는다.
- 실제 predictor가 `cuda:0`임을 확인했다. 기록된 CUDA allocated 최대92.91MiB,
  reserved172MiB, 프로세스 max_allocated133.51MiB였다. 마지막 값은 warm-up 포함
  가능하며 시스템 전체 GPU 점유나 BEVFusion 동시 실행 메모리가 아니다.

| 마지막 고정 구간 | p50 | p95 | p99 |
|---|---:|---:|---:|
| callback → 표준 출력 publish | 24.62ms | 28.25ms | 39.82ms |
| camera header → 출력 관측기 수신 | 25.09ms | 28.75ms | 40.94ms |

관측 cadence는 약8.26Hz였다. 최대10Hz 설정을 실제10Hz 처리 보장으로 쓰지 않는다.
독립 topic별10Hz logging throttle 때문에 helper/selection exact join은 일부만 남는다.
로그의 join 누락률을 실제 node의 packet loss로 표현하지 않는다.

v2에서는 Ultralytics outer model의 `device`를 읽어 CPU로 잘못 표시했고 CUDA 메모리가
null이었다. predictor가 모델을 복사하는 동작을 확인해 **진단만** predictor/backend
우선으로 수정했고 v3에서 확인했다. 실제 추론 device나 모델 계산을 바꾼 수정이 아니다.
v2의 null을 메모리0 또는 CPU 추론 확정으로 다시 쓰지 않는다.

네 새 pose의 오프라인939/939와 다른 실제 validation pose의 UNKNOWN을 함께 남긴다.
고정 사진의 높은 일치율만으로 임계값을 낮추거나 자동 승격하지 않는다. 다음은
D의 작은/어두운 crop 및 실제 가림·회피 시점에서 UNKNOWN의 지속과 정지·재출발을
확인하는 것이다. 이 시험으로 공식 황색 판단이나 실주행 전환 지연이 확정되지는 않는다.
confidence는4비트 각각의 `max(p,1-p)` 중 최솟값이지 좌화살표의 점등 확률이 아니다.
예컨대 p_left=0.21이면 꺼짐 쪽 확신이0.79다. UNKNOWN 임계값만 낮춰도 이 confident
false negative가 좌화살표 ON으로 바뀌지는 않는다.

## D의 작은 화살표 보강: CNN v5

실패를 확인한 뒤 기존 자료를 다시 읽었다. v4의 D native33(R+L) train 양성은
주로 `D2_18_offset_yaw`의 원본12장과 그 YOLO crop12개였다. val도20m 시점이었다.
train housing은 약61×20px, live29m는 detector proxy 약38.5×11.9px였다.
CNN resize 뒤에도 상대 housing 점유율이 달랐다. 반면 같은 val JPEG에서
E28→E37 detector의 crop 모서리는0~2px 차이여서 **검출 checkpoint 교체만이
큰 crop mismatch를 만들었다는 근거는 약했다.** 작은 D 화살표 자료 부족이
타당한 개선 가설이었지만 유일한 원인이라고 확정하지 않는다.

### 추가 자료와 독립 시점

- 새 train 시점 두 곳: 거리27.5m/횡+1.1m/yaw−3.5°,33m/횡−1.3m/yaw+4.5°.
  Foggy11/13/15 × R/R+Y/R+L/Y/G 각2장으로 총60개다.
- native052만 제어하고 JPEG 전후 실제 ID/상태·신선한 receipt·환경·P·SHA를
  확인했다.4개 crop sheet60개와6개 전신 사진을 root assistant가 직접 검수했다.
  원본의 작은 해상도 한계와 독립 사람 GT가 아니라는 사실은 그대로다.
- **CNN만 보강**했다. 이60장의 다른 화면 신호를 배경으로 놓고 YOLO를 재학습하지 않았다.
  기존 YOLO와 가중치 선택을 바꾸지 않았다.
- 기존792행/val168/test192를 값 그대로 보존하고60개를 train에만 추가해
  **852개 = train492/val168/test192**로 만들었다.
- v3 실패·기존939장을 본 뒤의 적응적 보강이므로 이 두 자료는 개발 회귀다.
  별도로31.7m/횡+0.4m/yaw+1.5°의 새 pose·6환경·동일5상태 각2장,
  **60개 holdout**의 계획을 v5 학습 전에 고정했다. 이 자료는 train에 넣지 않았다.
- holdout 원본은61장이다. 한 장은 요청R+Y 뒤 실제 native가R로 변해 제외하고
  bounded retry의 새 JPEG를 사용했다. 실패 원본을 지우거나 라벨을 바꿔 채우지 않았다.
 60개 crop와6개 전신 사진도 모델 결과를 읽기 전에 root가 직접 검수했다.

v5는 v4 이어학습이 아니라 같은 ImageNet 초기화·원래30epoch/batch16/seed0·
LR·증강·BN·FP32 recipe로 새로 학습했다. class pos_weight는 train 분포에서
다시 계산되는 기존 규칙을 따른다. 다만 v4는 Vast4090, v5는 로컬5060Ti이고
Pillow 등 runtime 차이도 있어 완벽한 단일변수 인과 ablation이라고 부르지 않는다.
val F1→동률 BCE로 E30을 고정했고 val168/168, 이후 기존 test192/192였다.
기존 test는 이미 개발에 노출된 회귀 자료이며 새 독립 test라고 부르지 않는다.

`yolo_cnn_cropmix_v5/best.pt` SHA256:
`3d29ee7f113c92f7eb7d7f4ddfe8a42c7b8d7094538d1b10f1b5fd709fdcda31`.
모델·threshold를 고정한 뒤 새 holdout60개의 같은 YOLO crop에 v4/v5를 한 번씩 적용했다.

| 새 D pose·6환경60개 | CNN v4 | CNN v5 |
|---|---:|---:|
| raw0.5 threshold 조합 일치 | 57/60 | **60/60** |
| runtime0.6 확신 조건까지 통과한 정답 | 54/60 | **60/60** |
| UNKNOWN | 4 | **0** |
| runtime 좌화살표 TP / 양성12 | 6/12 | **12/12** |
| runtime 좌화살표 FP / 음성48 | 0/48 | **0/48** |

UNKNOWN을 확실한 음성 또는 정답으로 바꾸지 않았다. v4는 확신 있는 arrow FN2개와
arrow 양성 UNKNOWN4개였다. 이 결과로 epoch/threshold를 다시 고르지 않았다.
새 pose지만 한 물리 head의 강제 현시60장이므로 자연 현시 전환·가림·전체 주행을
인증하지 않는다. 사전 고정·SHA·원본·QA는 `d2_foggy_domain_holdout_v1/`,
`d2_holdout60_crop_qc_v1/`, `d2_holdout60_v4_v5_comparison_v1/`에 있다.

### 개발 회귀와 실제 ROS 재확인

이전에 실패한 **동일 JPEG·동일 저장 crop**의 native 안정16개는 v5에서 모두
known+정답이었다. 특히 R+L9개는 v4의 known OFF7개/UNKNOWN2개에서 v5의
정답9개로 회복됐다. 좌화살표 확률은 약0.99998~0.99999였으나 calibrated probability로
주장하지 않는다. 기존 연속939개도 같은 저장 map ROI에서 정답939개/UNKNOWN0을
유지했고 좌화살표116개가 모두 맞았다. CPU에서 같은 adapter로 수행한 개발 회귀이며
새 독립 시험이나 GPU 지연 비교가 아니다. `yolo_cnn_v5_savedbox_regression_v1/`을 본다.

새 모델로 **실제 ROS D29·Foggy15 자연 현시45초**도 다시 실행했다.
`live_trial_supervised_v5_regression_v4/`에 원본42 JPEG·모든 로그를 남겼다.

- 고정 window의 selection349개 중346개가 연결됐다. 나머지3개는 모두
  `localization_uncertainty_exceeds_bound`이며 관측상 단발·앞뒤 회복이 확인됐다.
- classifier348개 중345개 known,3개는 `no_selected_crop`이었다.
  **좌화살표 저확신 UNKNOWN은0개**였다. 미선택3개를 분류 성공으로 세지 않는다.
- native 전후 안정40개 중 classifier와 **exact stamp**로 연결된15개만 채점해
  raw/known 모두15/15였고 R+L subset6개도 모두 맞았다. 연결되지 않은25개와
  전환 중2개를 보간하거나 성공으로 채우지 않았다.
- callback p50/p95/p99는25.49/37.65/47.49ms, camera header→출력 관측 수신은
  26.02/38.73/48.25ms였다. 두 프로세스 모두 cuda:0, 후보 SHA도 일치했다.
  같은 architecture의 로컬 정지 실측이며 다른 동시 CPU 작업·MORAI 부하 조건에서
  전후 지연을 엄밀히 비교한 benchmark는 아니다. 이 수치를4090 LAN 보장으로 쓰지 않는다.
- 촬영 wait exit0 뒤에만 소유 프로세스를 종료했고 최초 pose·Sunny13·정지/P를
  복원했다. 사용자 MORAI·roscore와 기존 production은 유지했다.

작은 D의 관측 실패를 끝까지 재현·보강·별도 시점·실ROS로 확인한 결과 v5를 선택했다.
여전히 실제 route helper를 쓴 폐루프, 가림/회피 중 대상 선택, 공식 황색 판정은
확인되지 않았다. v4 실패 기록과 v5 개선 기록을 함께 보존한다.

## 선택 후보 실행과 다음 단계

다음은 **이 PC에 저장된 연구 가중치**를 명시하는 실행 예다. 올바른 localization/TF,
카메라 입력, 실제 planning route가 먼저 있어야 한다. 개발 native API나 Ego route oracle을
운영 입력으로 요구하지 않는다. 기존 production 노드와 같은 이름으로 중복 실행하지 않는다.

```bash
cd ~/AjouNice2026
source base_ws/install_aw149/setup.bash
export AJOUNICE_ROOT="$PWD"
R="$PWD/artifacts/evaluation/traffic_light_robustness_261025"
ROS_NAMESPACE=/traffic_light_trial roslaunch traffic_light_detection traffic_light_node.launch \
  selection_mode:=map_association classifier_mode:=cnn detector_imgsz:=960 \
  weights_path:="$R/yolo_n960_final_v1/selected.pt" cnn_checkpoint_path:="$R/yolo_cnn_cropmix_v5/best.pt" \
  inference_python:="$HOME/miniconda3/envs/traffic_light_gpu/bin/python" device:=0 cnn_device:=cuda:0 \
  output_topic:=/traffic_light_trial/traffic_signals map_roi_topic:=/traffic_light_trial/map_rois \
  route_group_id_topic:=/traffic_light_trial/route_group_id publish_debug_image:=false
```

이 명령은 후보 출력만 만들며 실제 판단의 입력 remap이나 production 배포를 하지 않는다.
팀 PC에는 가중치 두 개와 호환 runtime을 별도로 전달해야 한다. 현재 Git pull만으로
새 모델이 설치된다고 안내하지 않는다.

1. 팀 정책으로 황색·R+Y·카메라 상실 시 동작을 확정한다. 인지 개선과 구분한다.
2. D의 UNKNOWN 및 실제 route helper를 포함한 LAN·4090 풀스택을 짧게 비교한다.
3. 결과가 통과하면 새 모델 배포·기본값 승격을 별도 수행한다. 실패 시 기존 모델/HSV로
   명시적으로 돌아갈 수 있으며 이번 작업은 기존 가중치를 덮어쓰지 않았다.

## 실행·보존 경계

- 격리 후보는 `/traffic_light_trial/...`에만 출력했다. 기존 운영 topic을 덮어쓰지 않았다.
- 새 모델·실험 JPEG·manifest는 Git 일반 파일로 넣지 않는다.
- CUDA/Torch 공용 환경을 바꾸지 않고 Vast 전용 venv로 CNN을 학습했다.
  느린 SSH 업로드 때문에 최종 YOLO와 v5 CNN은 로컬 GPU로 실행했다.
- 새 Drive 반출 승인이 필요한 경로는 보류하고, 승인된 Vast SSH만 사용했다.
- 실제 ROS 시험을 위해 traffic 패키지를 증분 설치했다. 오래된 localization/bringup
  설치본의 삭제된 v85 경로도 **현재 source에서 재설치만** 했으며 source는 수정하지 않았다.
- 단일 sensor-static-TF publisher를 사용했다. 같은 global node name을 두 launch에서
  중복 실행하면 서로 교체되는 것을 발견해 시험 구성을 정리했다.
- 녹화 후 최초 위치·Sunny13·신선한 정지/P를 확인하고 직접 시작한 gateway/hold만 종료했다.
  사용자가 켠 MORAI·roscore는 종료하지 않았다.

## 확인한 검증 범위

- 평가 도구·인지 후보·현재 정책 계약의 관련 Python 테스트 **428개 통과**.
  저장소 전체 또는 모든 과거 정책 기대값의 통과를 주장하지 않는다.
- 수정 Python 구문, launch/package XML, `git diff --check`, 인지 context 링크 검사를 통과했다.
  현재 신호등 CURRENT는100줄 이내를 유지하고 상세 근거만 이 기록에 둔다.
- traffic 패키지를 증분 설치했고 실제 설치본으로 최종 ROS 시험을 수행했다.
  모델과 자료는 ignored artifacts에 보존하며 code/docs만 커밋한다.
