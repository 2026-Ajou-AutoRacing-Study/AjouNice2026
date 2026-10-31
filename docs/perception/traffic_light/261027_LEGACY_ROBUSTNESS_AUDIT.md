# 옛 신호등이 덜 끊겨 보인 이유: 전체 파이프라인 감사

종류: EVIDENCE / 2026-10-27. Git 원본과 팀원 기록의 코드 대조다.
현재 운영 기준은 [CURRENT](CURRENT.md)이며 **옛 정책 복원·후보 승격 권고가 아니다.**
후속 상태: 이 감사 뒤 순수 UNKNOWN 유예0.3초만 별도 검증·승격했다([배포 정리](261027_UNKNOWN_GRACE_RELEASE.md)). 아래 현행 비교는 감사 당시 유예0 기준이며 옛 7/4·HSV 복원은 아니다.

## 결론

옛 강건성을 **7/4 하나의 효과로 설명할 수 없다.** 팀원 v1.1의 실제 경로는
`중앙 YOLO box → HSV → 구간별 램프 보정/fallback → 7/4 → D 관측 hold`였다.
현재의 `지도 identity → CNN → 관측 보존`과 입력·누락·stamp 계약이 다르다.
특히 D는 빈 입력을 투표에 넣지 않고 직전 관측을 최대 1.5초 유지했다.
새 관측 보존 7/4가 identity 상실 즉시 UNKNOWN이 되는 것과 같은 시험이 아니다.

이 차이는 덜 끊기는 **메커니즘**을 설명한다. 당시와 같은 영상·native GT·주행 상태의
동일 입력 대조가 없으므로 옛 실주행 안정성의 원인 비중이나 안전성을 인증하지 않는다.

## “v86 이전”도 세 시기로 나뉜다

| 시기 / 근거 commit | 실제 계약 | 해석 주의 |
| --- | --- | --- |
| 2026-08-20 `bc439bbb` | launch를 5/3→7/4로 변경. HSV·충돌 resolver 뒤의 빈 배열도 투표. 첫 6회는 현재 입력 passthrough | 아직 구간 profile/RED fallback/D 관측 가드가 없었다. 08-21 `b868c732`는 HSV MORPH_OPEN과 D의 R+G→R+좌화살표 보정을 추가했다 |
| 2026-09-14~29 `3844e97a`, `6921e63f`, `d84190bd`, `a1b5c4b8` | profile 도입 당시 fallback은 경로별 GO. 09-15 C/D를 RED로 바꾸고 D 관측 가드 추가·상한 수정. 09-29 A/B도 RED로 변경 | 초기에 덜 멈춘 것은 미검출을 GO로 만든 결과일 수도 있다. 이 시기를 모두 fail-closed로 묶으면 안 된다 |
| 2026-09-30 팀원 v1.1 `b272638a` | 맵 v85, 전 미션 RED fallback, C 램프 복원, D 1.5초 가드, 7/4. D 황색 순서 의존은 `c650beb1`에서 제거 | 제공 원문의 최신 v1.1이 지칭한 revision이다. 앞부분 v1.0·미구현 제안과 구분한다 |

v1.1 원본 근거는 [팀원 원문](../../../artifacts/reference/traffic_light_team_notes_261025/source.md)
716~721행이다. 보존·SHA와 검증 한계는 [기존 대조 기록](261025_TEAM_NOTES_AND_ROBUSTNESS_AUDIT.md)에 있다.

## 무엇이 달랐나

아래 “옛 코드”의 행 번호는 모두 `b272638a`의 해당 파일 기준이다.

| 요소 | 옛 전체 경로 | 현재 직접 관측 / 별도 stamped 후보와의 차이 |
| --- | --- | --- |
| 대상 선택 | `light_color_utils.py:131–142`: 중앙 ±20% 후보에서 `abs(cx-.5)+.6*cy` 최소. 중앙 후보가 없으면 전체 box에서 선택 | map/TF/covariance/head identity 거부가 없었다. 다른 head를 계속 읽어 누락이 적어 보일 수 있지만 올바른 대상이라는 증거는 아니다. 현재는 지도 자격·identity 실패를 분리한다 |
| 분류 | 같은 파일 `105–126`: crop 3배 확대, HSV 색 픽셀 5개 이상이면 램프 출력. 녹색 형상은 `66–102`의 원형도에 따라 원형/좌화살표 | CNN bit-confidence UNKNOWN과 다른 입력이다. HSV의 더 적은 abstention이 더 높은 정확도를 뜻하지 않는다 |
| C 램프 복원 | `traffic_signal_profiles.yaml:38–46`: G 또는 좌화살표 단독을 모두 G+좌화살표로 합침. R+G/R+좌화살표는 R로 보정 | 여러 raw 조합이 하나의 투표 key가 되어 흔들림을 줄였다. 현재는 보이지 않은 G·화살표를 추가하지 않는다 |
| D 보정 | YAML `60–64`: 좌화살표 단독→R+좌화살표, Y 단독→R. R+Y는 별도 유효 조합 | 색 인식과 경로 정책이 섞여 있었다. 현시가 안정돼 보이는 것과 원시 램프 인식 정확도를 분리해야 한다 |
| A/B/C 누락 | `signal_phase_policy.py:92–127,159–171`: 빈 입력→RED fallback도 7/4에 투표. 표 갈림도 `apply_transition`에서 RED | 안정 7G 뒤 누락 1~3회는 G, 4번째는 RED. 지금의 빈 관측 UNKNOWN과 의미가 다르다. `:fail_open`이라는 옛 사유 이름도 실제 GO를 뜻하지 않는다 |
| D 누락 | 같은 파일 `172–206`: fallback 입력을 투표에서 제외, 최근 관측 출력 최대 1.5초 유지. 황색 포함 출력은 별도 4초 상한 | D의 단절 완화는 7/4와 별개인 last-observed hold다. 현재 직접 경로의 제한된 공백 완충과 새 stamped 후보의 identity-loss reset을 동일시하면 안 된다 |
| 시간·중복 | `signal_smoother.py:26–40`: payload가 램프+confidence이고 distinct-image 중복 제거·A/B/C 원본 age 상한이 없음. D만 내부 관측 clock을 별도로 사용 | 새 후보는 원본 stamp 중복을 제외하고 aged window를 제한한다. sparse 입력에서 같게 동작하지 않는다 |
| wire stamp | `traffic_light_node.py:313–315`에서 현재 JPEG stamp를 먼저 설정하고, 과거 winner/hold를 골라도 변경하지 않음 | 옛 결과는 현재 JPEG stamp로 재발행됐다. 원본 winner stamp를 wire에 보존하기 시작한 변경은 2026-10-26 `b2e22dd0`; 현재는 반복 winner를 새 관측으로 위장하지 않는다 |
| 미발행·debounce | node `276–291,332–366,417`: rate-limit/디코드 실패는 return. 그러나 YOLO box 없음·빈 crop인 정상 callback은 정책 처리 후 계속 publish | “못 읽으면 아예 안 보내서 무조건 마지막 값 유지”는 이 revision의 호출경로와 다르다. traffic 노드의 별도 연속-confirm debounce도 없으며 다수결과 D hold를 구분해야 한다 |

옛 `SignalSmoother`는 **연속 4회 확인이 아니라 최근 7회 중 같은 전체 램프 조합 4표**다.
첫 6회 passthrough도 실제 구현이다. 새 후보의 cold-start passthrough는 이를 명시적으로
시험하지만, 원본 stamp·age·identity·관측 보존 계약까지 옛 것으로 되돌리지는 않는다.

## 옛 코드를 직접 실행해 분리한 효과

`git show b272638a:<path>`로 읽은 policy·smoother·YAML을 메모리에서 실행했다.
ROS·MORAI·설치·원본 파일은 변경하지 않았으며, 이는 합성 입력의 코드 동작 재현이다.

| 동일한 코드 입력 | 확인한 결과 |
| --- | --- |
| A/B에서 안정 7G 후 10Hz 빈 입력 | 누락 1~3회 G, 4번째 RED. 누락도 RED 표로 창을 밀어낸다 |
| D에서 안정 7개의 R+좌화살표 후 빈 입력 | 누락 4회에도 R+좌화살표 유지. 1.4초까지 유지, 1.6초에는 RED·queue reset. 누락 때 queue는 전혀 진행하지 않았다 |
| C의 G / 좌화살표 / G+좌화살표 | 셋 모두 G+좌화살표로 정규화. raw 조합 변동이 다수결 전에 사라졌다 |

D의 “못 보지만 잠시 계속 진행”을 설명할 때 가장 직접적인 옛 메커니즘은
**1.5초 관측 hold와 새 JPEG stamp 재발행**이다. 이를 현재에 그대로 복원하면 실제
꺼짐·신호 전환·시야 이탈까지 과거 허가로 덮을 수 있으므로 단순 복원하지 않는다.
지속 누락을 숨기는 것과 올바른 현재 램프를 계속 읽는 것은 다르다.

## 당시도 완전히 강건했던 것은 아니다

### 판단도 현재와 달랐다

`3844e97a`의 competition fail-open과 `ef9a1583`의 실제 experimental 모듈을 대조했다.
그때의 신호 인지와 현재 guard를 섞어 실행하면 아래와 같은 옛 제어 결과가 자동 재현되는 것은 아니다.

| 판단 요소 | 가드 도입 전 experimental 경로 | 현재 guard ON |
| --- | --- | --- |
| 빈 배열·UNKNOWN | `competition_fail_open=true`이면 fresh UNKNOWN도 GO | 첫 색 관측 이후 fresh UNKNOWN은 STOP. 미수신/timeout GO는 별도 유지 |
| 황색 | `yellow_light_is_go=true`이면 바로 GO | 황색 master OFF 기본이면 STOP. ON 예외는 별도 계약 |
| STOP 진입 | 이미 STOP이 아니면 `stop_time_hysteresis=0.1`초를 넘긴 뒤 정지 요구 | guard가 이 진입 지연을 우회하여 즉시 반영 |
| 통과 판단 | 이전 STOP이 아니고 물리 운동 조건을 만족하면 `isPassthrough`가 정지점 삽입을 건너뛸 수 있었음. 해당 STOP이 Y인지 직전 G에서 바뀐 것인지 직접 제한하지 않음 | 신선한 허가 G→Y 등으로 제한. 기본 황색 OFF에서는 비활성 |

따라서 당시의 덜한 제동에는 **UNKNOWN·황색·짧은 STOP에 반응하지 않았던 정책**도 기여할 수 있다.
반대로 이를 전부 되살리면 신호 준수를 악화시킬 수 있다. 과거 bag의 실제 설치 값·입력까지
확정하지 않은 상태에서 이 기여도를 수치로 분배하지 않는다.
planning 본체에 남은 UNKNOWN→지난색 cache는 해당 모듈의 설명 근거가 아니다.
과거와 현재의 `findValidTrafficSignal()`은 모두 `getTrafficSignal(id, false)`의 raw-map을 읽는다.

```bash
git show ef9a1583:planning_ws/src/autoware_behavior_velocity_traffic_light_module/src/experimental/scene.cpp | nl -ba | sed -n '118,247p'
git show ef9a1583:planning_ws/src/autoware_behavior_velocity_traffic_light_module/src/experimental/scene.cpp | nl -ba | sed -n '330,418p'
```

현재 입력에서 이 원리를 제한해 재사용하는 시험은
[짧은 UNKNOWN 완충 후보](261027_UNKNOWN_GRACE_DRIVING_COMPARISON.md)에 별도로 기록한다.

- 팀원 v1.1 원문 **968행**은 접근 중 최대 **15초 연속 fallback RED**를 미해결로 적었다.
  문서의 M2 표도 C/D의 화각 이탈 뒤 RED 지속·DNF 위험을 기록한다. “예전은 모든 누락을 해결”한 근거가 아니다.
- 6,209프레임·D 97.2→97.5%는 당시 라벨/정책 회귀 주장이다. 제공 ZIP에는 그 라벨 원본이
  없어 새로운 CNN 정확도나 공식 주행 성능으로 검증할 수 없다.
- `b868c732`의 HSV 개선율은 그 당시 자료·분류 조건의 기록이다. 현재 YOLO/CNN의 독립
  housing/램프 평가와 분모·GT·환경·후처리가 달라 수치를 직접 비교하지 않는다.
- 옛 영상의 box/텍스트가 최종 정책 색인지 raw 관측 색인지도 확인해야 한다.
  원문 v1.1의 디버그 절은 이전 최종색-only 표기와 관측→최종 표기의 차이를 명시한다.

## Git 원본 재확인 명령

행 번호는 현재 파일이 아니라 아래 revision의 `git show` 출력에 적용한다.

```bash
git show bc439bbb -- base_ws/src/perception/traffic_light_detection/launch/traffic_light_node.launch
git show b272638a:base_ws/src/perception/traffic_light_detection/scripts/traffic_light_node.py | nl -ba | sed -n '276,426p'
git show b272638a:base_ws/src/perception/traffic_light_detection/scripts/signal_phase_policy.py | nl -ba | sed -n '92,206p'
git show b272638a:base_ws/src/perception/traffic_light_detection/config/traffic_signal_profiles.yaml | nl -ba | sed -n '18,76p'
git show b272638a:base_ws/src/perception/traffic_light_detection/scripts/light_color_utils.py | nl -ba | sed -n '60,142p'
git show b272638a:base_ws/src/perception/traffic_light_detection/scripts/signal_smoother.py | nl -ba | sed -n '16,46p'
git show a1b5c4b8 -- base_ws/src/perception/traffic_light_detection/config/traffic_signal_profiles.yaml
git show b2e22dd0 -- base_ws/src/perception/traffic_light_detection/scripts/traffic_light_node.py
```

개선 비교에서는 현재 영상의 누락을 **선택/identity 상실, CNN abstention, 실제 색 전환**으로
먼저 나눈 뒤, 투표만의 효과와 공백 hold의 효과를 별도로 검증한다. 옛 전체 경로를
통째로 되살리거나 이 감사만으로 운영 기본값을 바꾸지 않는다.
