# MORAI K-City 동적 객체 검증 시나리오

`kcity_2025_dynamic_validation.tar.gz`에는 MORAI Standard의
`R_KR_PR_K-city_2025` 시나리오 폴더에 넣어 사용할 JSON 1,489개가 들어 있다.
무결성은 `cd configs/scenario/morai_kcity_dynamic_validation && sha256sum -c SHA256SUMS`로
확인할 수 있다.

- 보행자/동물/자전거: 1,260개
- 좌·우 NPC 차량: 216개
- 원본 템플릿: 11개
- 검증 매트릭스 인덱스: 2개

## 설치

MORAI Launcher가 설치된 PC에서 저장소를 받은 뒤 압축을 기존 K-City 시나리오
폴더에 푼다. 같은 이름의 생성 파일이 있다면 압축 해제 전 백업한다.

```bash
cd ~/Desktop/AjouNice2026
MORAI_SCENARIO_DIR="$HOME/Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
tar -xzf configs/scenario/morai_kcity_dynamic_validation/kcity_2025_dynamic_validation.tar.gz \
  -C "$MORAI_SCENARIO_DIR"
```

MORAI의 `Scenario Load`에서 JSON 파일을 불러오거나, ROS 검증 실행 시
`MORAI_SCENARIO_DIR`에 위 경로를 지정한다.

```bash
MORAI_SCENARIO_DIR="$MORAI_SCENARIO_DIR" \
  scripts/runtime/run_morai_scenario_validation.sh \
  --velocity-kph 40 \
  --scenario dynamic_car_left_obj30_arrow_tip_ego40_pos001.json
```

파일명의 `ego40`은 MORAI JSON 속도가 아니라 플래너 실행 속도다. 따라서
`ego40` 파일은 항상 `--velocity-kph 40`과 짝지어 실행한다.

## 파일명 규칙

보행자/동물 시나리오:

```text
dynamic_<actor>_loc<01|02>_obj<speed>_<front|opposite>_crosswalk_ego<speed>_act<7|10|15>_pos001.json
```

NPC 차량 시나리오:

```text
dynamic_car_<left|right>_obj<speed>_<before_stopline|arrow_tip|first_dashed_line>_ego<speed>_pos001.json
```

차량 트리거는 `1_trigger=정지선 이전`, `2_trigger=화살표 끝`,
`3_trigger=첫 번째 점선`으로 정의한다. 생성·수정은 저장소의
`scripts/runtime/generate_morai_*_scenarios.py`를 사용한다.
