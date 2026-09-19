# Sunny 13시 production v4 생성기 검증

## 목적

v3 전체 180장면 수집의 실제 실패 양상과 카메라별 객체 분포를 반영한 다음
세대 생성기다. 기존 v3는 당시 수집의 재현 자료로 보존하며 v4가 덮어쓰지
않는다.

> 2026-08-22 재검증에서 당시 격리 차량 6종이 interleaved control/probe
> 두 위치 모두 정상 생성됐다. 아래 2번은 당시 판단을 설명하는 역사적
> 기록이며 현재 자동 생성 정책에서는 quarantine을 해제했다.

## v3에서 확인한 문제와 v4 조치

1. 객체가 없는 9개 negative scene의 LiDAR `Instance.txt`가 0바이트라는
   이유로 실패 처리됐다. v4 runner는 객체 수가 0일 때에만 빈
   `Instance.txt`/`Instance_8Points.txt`를 정상 라벨로 허용한다. 이미지와
   point-cloud binary는 계속 non-empty를 요구한다.
2. 서로 다른 링크에서도 반복 누락된 차량 6종은 개인용 런처 무인수집 풀에서
   격리한다. catalog에는 남아 있으므로 수동 재시험은 가능하다.
3. v3는 모든 actor를 최초 Ego 위치 기준으로 배치해 Ego가 약 9 m 전진한
   뒤에는 후방 카메라 객체가 많아졌다. v4는 주행 궤적 중간인 4.5 m 앞을
   배치 기준점으로 쓴다.
4. 보행자·혼합·장애물 장면에는 45% 확률의 전방 우선 후보를 적용한다.
   차량 중심 장면은 강제 좌우 균형을 만들지 않고 MGeo 도로 표본의 자연스러운
   우측통행·대향차·합류 관계를 유지한다.
5. 각 actor metadata에 배치 기준점 좌표계의 종·횡 위치, front/side/rear,
   차량 진행방향 관계를 기록해 이후 분포 감사를 재현 가능하게 한다.

## 생성 및 정적 검증 결과

- scenario 180개, split 126/27/27
- 총 actor 1,434개, 고유 asset 88종
- 격리 asset 배치 0개
- 배치 구역: front 712(49.7%), side 211(14.7%), rear 511(35.6%)
- 차량 진행관계: 같은 방향 298(53.0%), 대향 194(34.5%), 교차·합류
  70(12.5%)
- scene type과 topology 수는 v3와 동일
- 생성 과정에서 모든 JSON을 다시 읽어 actor 수, 28 m 범위, 링크 ID와
  정적 충돌 회랑을 검증함

이 수치는 “전방만 과도하게 채운 합성 장면”이 아니라 자연스러운 도로 관계를
유지하면서 초기 Ego 기준의 후방 편향을 줄인 절충이다.

## 생성 명령

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_sunny13_180_production_v4_set.py
```

출력은 다음 경로에 별도로 생성된다.

```text
dataset/morai_od/scenarios/sunny13_180_production_v4/
dataset/morai_od/metadata/sunny13_180_production_v4/
dataset/morai_od/manifests/sunny13_180_production_v4_sunny_t13.json
dataset/morai_od/reports/sunny13_180_production_v4_generation_summary.json
```

## MORAI 설치와 6장면 smoke test

전체 수집 전에 여섯 scene type을 하나씩 실제 로드·주행·GT 저장 검증한다.
`s13p4_012`는 빈 negative 라벨 허용 로직까지 확인한다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/generate_sunny13_180_production_v4_set.py \
  --install-dir \
  "$HOME/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
```

`roscore`, `rosbridge`, MORAI Ego Network 연결과 Capture Mode 설정 후:

```bash
cd ~/AjouNice2026

source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
source ~/morai_personal_ws/devel/setup.bash

python3 dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest \
  dataset/morai_od/manifests/sunny13_180_production_v4_sunny_t13.json \
  --capture-dir /home/luv/morai_cap/sunny13_v4_smoke6 \
  --motion-mode rolling-grouped \
  --scenario s13p4_001 \
  --scenario s13p4_002 \
  --scenario s13p4_003 \
  --scenario s13p4_012 \
  --scenario s13p4_013 \
  --scenario s13p4_029 \
  --scenario-load-attempts 1 \
  --continue-on-scene-error
```

판정 기준은 6개 scene 완료, scene당 6 frame, 모든 frame의 Camera 1~4와
LiDAR 필수 artifact 존재다. 이는 정적 생성 검증이 확인할 수 없는 MORAI
실제 asset load와 센서 저장을 확인하기 위한 최소 smoke test다.
