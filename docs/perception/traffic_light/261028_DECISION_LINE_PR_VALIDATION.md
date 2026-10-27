# 판단선 v2 PR 검증과 제동거리 한계

종류: EVIDENCE / 2026-10-28. 운영 기본값은 [CURRENT](CURRENT.md)를 따른다.

## 범위와 결과

- `dc2a71ec` 계열 후보, 명시적 지도v97, 실제 localization+od_gt+ROS1 planner/PP·PI로 시험했다.
- `braking_stop`, `braking_boundary` 두 실행은 A→B→C→D 전체 통과와 최종 목표 SUCCESS,
  정상 종료·실제 정지/P 확인이다. 시험 명칭과 달리 두 실행의 제어 황색은 **GO 분기**를 검증했다.
- 기존 차량 heading과 유한 정지선의 교점은 곡선 접근 중 늦게 생겼다.
  요청35m가 실제 A23.15m/B22.07m에서 발동한 것을 기록했으며 STOP 성공으로 세지 않았다.
- `braking_probe`는 동일 풀주행 출발점에서 B 제동을 측정하고 기존 세 번째 waypoint에서 안전 종료했다.
  이 실행은 D까지의 전체 주행 검증이 아니다. 시험용 ABC 인지 시작60m, D30m,
  B만 G→Y3초→R10초→G, planner의 신선한 정차 목표30m를 전환 기준으로 사용했다.
  운영 ABC40m/D30m는 그대로다. A 정지 대기는 명시적·조회 확인된 G로 해제했다.

## 실제 황색 STOP 측정

| B에서 최초 황색 STOP | 값 |
|---|---:|
| 실제 Ego 속도 | 15.74m/s (56.68km/h) |
| 당시 planner 계산 제동거리: b8/J16/응답지연0 | 19.55m |
| 당시 Lanelet 정차 목표 잔여 거리 | 24.23m |
| 실제 정지까지 Ego 속도 적분 | 23.25m |
| 목표 잔여량 변화로 본 이동 | 약24.25m |
| 실제 정지 시 목표 거리 | −0.024m (약2cm 초과) |
| 정지 소요 / 300ms 구간 최고 감속 | 3.10초 / 약−8.61m/s² |
| 카메라 header age p95 / 표본 최대 간격 | 0.177초 / 0.109초 |

10Hz 영상 근거의 Ego 속도 적분과 localization/path 거리 변화는 약1m 차이가 있으므로
정밀 액추에이터 계측으로 해석하지 않는다. 정지 요청 뒤 실제 정차까지 STOP은 유지됐다.
영상87.4초부터 브레이크0.0038의 약한 감속이 먼저 있었지만 신호 판단은GO였다.
당시 기존 궤적의 목표속도가 낮아지고 있었으며 세부 제한 모듈의 인과는 이 기록으로 확정하지 않는다.
88.2초 황색을 읽고 목표24.23m>계산19.55m의 판단선 밖에서STOP을 선택했다.
즉 d=D에서 감속을 시작한 것이 아니다. 후속 영상 `braking_probe/front_video_reason/`에는
실제 신호 판단 reason·판단선 안/밖·브레이크 명령을 표시한다. 이는 전체 스택 감속 원인 추정이 아니다.
일반 적색 B 제동에서도 계산19.91m 대비 적분27.81m였으나, 이른 정차 목표를 향한
완만한 감속이 포함되어 있으므로 그 숫자를 최대 제동능력으로 사용하지 않는다.

**결론:** 8/16/0은 이상화된 계산이며 현재 스택의 현실적인 정지거리 보장으로 승인하지 않는다.
황색 STOP 표본에서는 속도 적분 대비 약19% 짧았다. 초기 가속도0·응답지연0 가정,
인지/계획/제어 지연과 일반 제동 제한6/10의 차이가 남는다. jerk가 지연을 대신하지 않는다.
설정6/10은 해당 속도에서 식상 약25.29m지만, 새 실주행 검증값이 아닌 비교 계산이다.
임의 계수를 한 표본에 맞춰 승격하지 않았으며 **황색 master 기본 OFF**를 유지한다.

## 판독과 시험 제어의 한계

정차 중 93.9초에는 native GET은 R 전후였지만 **현재 카메라에 실제 G가 렌더링**됐고
CNN도 G를 읽었다. 따라서 이 표본을 CNN 적색→녹색 오판으로 집계하지 않는다.
약0.1초 GO 후 R STOP으로 복귀했고 차량은 정차 상태였지만, 강제 SET/GET과 렌더의
일치·짧은 허가의 일반 안전성을 인증하지 않는다. 요청색 또는 GET만으로 영상 GT를 만들지 않는다.

## 통합·재현

- 최신 planning `784713d5`는 세 실행의 정지/P 확인 **후** 전체 병합했다.
  병합 후 기본지도v98·RSS/제어 변경의 새 실주행 인증은 아니다.
- 병합 후 관련 Python167개 통과·1개 생략, native C++117개 통과,
  신호등 및 관련 Python/launch 증분 설치·scoped source/install 계약 검사 통과.
  문서 검사 잔여1개는 기존 `kcity_map/README.md`의65줄 예산 초과다.
- 영상은 `artifacts/evaluation/traffic_decision_line_261028/{braking_stop,braking_boundary,braking_probe}/front_video/front_evidence.mp4`.
  첫 두 영상은 D통과+5초, 마지막은 짧은 제동 시험 종료까지다. 신규 영상의 GO/STOP은
  별도 읽기전용 planner 토픽이며 예전0.5초 로그 재편집본이 아니다. 기존 planner 콘솔은 유지한다.
- 원본·bag·영상·observer/phase_controller/실행 receipt는 같은 평가 및 validator 경로에 보존한다.
  모델·지도 파일·기존 수집 snapshot을 수정하지 않았다. PR은 기존 #60을 갱신하며 자동 병합하지 않는다.
- 후속 BEV 합본은 `braking_probe/front_planviz/front_planviz.mp4`다. 실제 녹화된v97 Planviz
  viewport(461,401,519,511)만 잘라 전방 오른쪽에 붙였다. 별도 화면 녹화 시작을 전방14.85초에
  근사 정렬했으며 정밀 지연·프레임 동기 인증은 아니다. 전방 시각/픽셀/로그는 유지하고
  BEV가 없는 처음/끝은 없음으로 표시한다. 재사용 도구는 `scripts/evaluation/compose_traffic_front_planviz_video.py`다.

<a id="calibration-261028"></a>
## 후속: 실제 제어 제한과 6/10 비교 (2026-10-28)

이 절은 위8/16 시험과 최신 planning784713d5 통합 **이후**의 추가 근거다.
인지·판단 상태기계·지연0·지도v97·원본 채점선은 유지하고 계산용b/J만6/10으로 명시했다.
로컬 MR, localization+od_gt, 실제 ROS1 planner/PP·PI이며 BEVFusion·LAN/4090 부하 인증은 아니다.
시험용 내부 속도 상한만10/16m/s로 설정했고 throttle/brake/gear를 시험 도구가 직접 주지 않았다.
고유 sender의 상한은 정지/P 뒤 해제하며 다른 sender를 clear하지 않는다.

| B 황색 STOP | 저속 | 고속 |
|---|---:|---:|
| 첫 STOP 영상 시각 | 105.3초 | 100.0초 |
| 실제 Ego 속도 | 35.75km/h | 55.88km/h |
| 계산 D (b6/J10/지연0) | 11.18m | 24.80m |
| STOP 요청 시 Lanelet 목표 잔여량 | 11.53m | 25.31m |
| 정차까지 Ego 속도 적분 | 10.86m | 25.16m |
| 같은 구간 목표 잔여량 변화 | 11.69m | 24.88m |
| 최종 목표 잔여량 (+선 전/−초과) | −0.16m | +0.43m |
| 정차 소요 | 2.20초 | 3.40초 |
| 카메라 age p95 | 0.158초 | 0.195초 |

둘 다 현재 카메라에서 녹색을 여러 번 읽은 뒤 황색 STOP을 요청했고 정차까지 STOP이 유지됐다.
시험 제어기 기록의 직전 연속 허가 관측 수는19/4개다. 저속에는 그보다 이른 GO/STOP 변동도 있어
전체 영상이 무변동이었다고 주장하지 않으며, 표에는 연속 황색 STOP 구간만 썼다.
두 실행은 SUCCESS와 실제 정지/P를 확인했다. 목표는 기존 세 번째 waypoint이며 D 완주 시험은 아니다.
강제 신호 시험이며 고속 A 대기 해제 SET, 저속 종료 뒤 D SET도 별도 기록했다.
native 요청/조회값을 픽셀 GT나 운영 입력으로 사용하지 않았다.

실행 snapshot의 실제 BVP 제한은−6m/s²·−10m/s³였다. 제어기 허용 최대8은 즉시8을
요청한다는 뜻이 아니며 brake 상승률2/s·가속도 LPF0.1초도 있다. 저속 제어 debug의
목표 감속은 약−5.8m/s², 실제 감속은 순간 더 강했다. 이 때문에8/16 계산과 실제 정지가
다를 근거가 있으며, 단순히 GPU/CPU 지연 때문이라고 단정하지 않는다.

**판단:** 계산용 기본을6/10으로 맞추되 황색 master OFF와 응답지연0은 유지한다.
launch·validator·normal/experimental C++ 기본값을 통일했으며 실제 planner/controller 제한은 바꾸지 않았다.
normal planning profile은 별도로2.8/5이며 이번 보정의 검증 범위가 아니다. 그 profile에
황색 ON을 적용하려면 별도 설정·제동 확인이 필요하다. 기존 YAML의 명시값·frozen snapshot은 보존한다.

속도 적분과 localization/path 거리는 저속에서 약0.83m 달랐고 고속 계산도 적분보다0.36m 짧다.
따라서6/10도 정지거리의 엄밀한 상한·판단선 등호에서의 정지·적색 미위반 보장은 아니다.
동일 B 구간 두 속도의 개발 검증이며 계수를 추가로 한 표본에 맞추지 않았다.
더 큰D는 늦은 STOP 위험을 줄이는 대신 GO확정을 더 일찍 만들 수 있으므로 단순한 안전성 향상으로 읽지 않는다.
중간 속도13m/s 추가 실행은 trajectory75초 대기 초과/재요청 거절로 출발 전 MANUAL_ABORT했고
정지/P 확인·제어 생존 프로세스0을 확인했다. 제동 검증 성공 표본에 넣지 않았다.

- 자료: `artifacts/evaluation/traffic_braking_calibration_261028/{low,high}/`의
  `braking_B_final.json`, `front_video/frames.jsonl`, phase events 및 `supervisor_{low,high}/summary.json`.
- validator 실행: `calibration_6_10_low_261028_261028_055705`,
  `calibration_6_10_high_261028_261028_060006`, 실패 `calibration_6_10_mid_261028_261028_060258`.
- 고속 새 합본: `high/front_planviz/front_planviz.mp4`. 실제 녹화v97 viewport만 합쳤고
  시작 offset14.85초는 근사다. 정밀 시간 대조는 원본 ROS stamp/bag을 사용한다.
- 보정 후 관련 Python174개(실제 source/installed launch 해석 포함)와 Native120개 통과.
  Native는 별도 ROS master에서 실행했고 시험 master도 종료했다. C++ plugin·validator·launch
  증분 설치 후 scoped source/install 계약 검사를 통과했다. 문서 검사의 기존 지도 README
  줄 수 초과1개는 그대로 남기며 이번 신호등 보정의 성공으로 숨기지 않는다.

## 후속 배포 결정: 황색 ON·최신 planning 통합

위 보정 직후에는 master OFF였으나, 후속 사용자 요청으로 기본 ON으로 전환했다.
계산6/10·지연0·실제 제어 제한·인지 모델·완충은 유지한다. C++ normal/experimental,
module YAML, planning/full-stack launch, validator 새 실행·GUI 기본을 동일하게 ON으로 맞췄다.
명시적 false와 기존 frozen 실행은 보존하며, 첫 관측 황색·정차 중 황색의 예외 출발 금지도 유지한다.
확정 뒤 적색을 무시하는 합의 정책과 제동거리 상한 미보장 등의 기존 한계는 사라지지 않는다.

최신 planning600125b1(MPT horizon 축소 시 근접 경로 연속성 보완)을 전체 merge했다.
별도 지도 변경은 없으며 최신 v98을 pinned v97에서 재현해 SHA·metadata를 검사했다.
새 자산을 임의 v99로 올리거나 정지선·경계를 다시 이동시키지 않았다. 이 통합 후 추가 폐루프 주행은 하지 않았다.

ON 전환 후 집중 Python187개·신호등 Native120개·MPT horizon5개·지도 도구34개를 통과했다.
관련 패키지를 증분 설치했고 설치 트리103개 지도 참조와 v98 route/SHA 계약, 신호등 scoped
source/install 검사도 통과했다. 이전 설치본에만 남았던 v85 speed editor launch 한 개는
`install_aw149/share/bringup/legacy/launch/`로 옮겨 보존했고 활성 경로에서는 제외했다.
광역 validator 검사는476개 통과·2개 실패다. 기존 `test_full_course`의 경유점 목록과
`test_mission_starters`의 roundabout manifest SHA 불일치이며, 관련 mission 파일을
이번 ON 전환에서 바꾸거나 검사를 완화하지 않았다. 집중 신호등 검사 통과와 구분한다.

원격 직접 병합cda35545도 확인했다. 부모는600125b1과우리22a5f87c이며, tree SHA가
로컬 통합aa8b7b51과동일했다. 잘못된 덮어쓰기나 별도 충돌 수정은 없어서 revert하지 않았다.
그 병합을 로컬에도 가져왔고 파일 내용 변화0을 확인했다. 최종 PR은 이미 들어간 인지/ROI/6·10
전체를 새 변경으로 중복 주장하지 않고, 남은 기본ON·GUI·검사·현재 문서 변경을 반영한다.

## 팀 PC 정적 검사 불일치 후속 수정

`e46797b7`의 기본 ON 전환 때 별도
`traffic_light_detection/test/test_traffic_light_production_defaults.py` 두 곳은
OFF 기대값으로 남았다. 위 집중187개 검사에는 이 파일이 포함되지 않았다.
팀 PC 보고와 동일하게 로컬에서도 수정 전 **2실패·2통과**를 재현했다.
호스트·GPU 차이가 아니라 누락된 회귀 테스트 갱신이며, 이 실패만으로 신호등 노드의
기동 실패·황색 오출발·주행 안전성을 판단할 수 없다.

두 기대값을 승인된 ON 계약에 맞추고, planning/통합 stack 각각의 기본 ON과 명시적
`enable_yellow_passthrough:=false` 전달을 실제 read-only launch 해석으로 검사한다.
노드·ROS master는 시작하지 않으며 실행 코드·모델·지도·정책 기본값은 수정하지 않았다.

```bash
AJOUNICE_TEST_LAUNCH_DUMP=1 python3 -m pytest -q \
  base_ws/src/perception/traffic_light_detection/test \
  base_ws/src/planning_addons/ajounice_planning_integration/test/test_signal_yellow_braking_launch.py \
  scripts/runtime/test_traffic_light_unknown_grace_release.py
AJOUNICE_TEST_LAUNCH_DUMP=1 AJOUNICE_TEST_LAUNCH_TREE=installed python3 -m pytest -q \
  base_ws/src/perception/traffic_light_detection/test/test_traffic_light_production_defaults.py
```

결과는 소스 범위 **445통과**, 설치 트리 대조 **8통과**다. 사용자 검토 후 이 테스트·기록
수정만 별도 PR로 planning에 반영한다. 실행 코드 변경이 없으므로 재빌드는 필요 없다.
