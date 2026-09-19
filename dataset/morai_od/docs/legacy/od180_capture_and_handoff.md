# MORAI OD180 환경별 캡처와 학습자 전달

## 1. 데이터 식별 단위

30개 Internal Scenario JSON이 base geometry다. 동일 base를 다음 여섯
환경에서 반복해 180 capture run을 만든다.

```text
base001
├── sunny_t11
├── sunny_t13
├── sunny_t15
├── foggy_t11
├── foggy_t13
└── foggy_t15
```

각 run은 저속으로 연속 이동하며 6회 trigger한 뒤 MORAI 공통 timestamp로
묶인 6프레임을 저장한다. 전체는 1,080 multi-sensor sample이다.

학습 split은 이미지나 환경 폴더가 아니라 `base_scenario_id`로 결정한다.
같은 base의 여섯 환경과 모든 frame은 반드시 같은 split에 둔다.

- train: base 20개, 720 sample
- val: base 5개, 180 sample
- test: base 5개, 180 sample

## 2. 캡처 저장 구조

환경별 실행은 서로 다른 하위 폴더에 저장한다.

```text
/home/luv/morai_cap/od180_v1/
├── captures/
│   ├── sunny_t11/<timestamp>_run/
│   ├── sunny_t13/<timestamp>_run/
│   ├── sunny_t15/<timestamp>_run/
│   ├── foggy_t11/<timestamp>_run/
│   ├── foggy_t13/<timestamp>_run/
│   └── foggy_t15/<timestamp>_run/
├── handoff_manifest.json
├── samples.jsonl
└── handoff_metadata/
```

센서 파일은 MORAI sensor 디렉터리를 유지한다. 파일명에는
`base scenario + environment + frame + 실행 ID`가 들어간다.

```text
base007_foggy_t13_f02_r0807..._Intensity.png
```

환경별 폴더와 파일 prefix 중 하나만으로도 구분할 수 있지만 둘 다 남겨
수동 복사 이후에도 provenance를 잃지 않게 한다. 시나리오별 폴더를 180개
만들 필요는 없다. `samples.jsonl`이 각 sample의 Camera 4대·LiDAR·GT
파일을 묶어 준다.

## 3. 실행 전 원칙

1. MORAI에서 해당 batch의 날씨와 시간을 수동으로 설정한다.
2. 첫 scenario를 수동으로 한 번 로드해 환경 상태가 유지되는지 확인한다.
3. Camera 4대와 LiDAR Capture Mode 및 GT 출력을 켠다.
4. Simulator Network와 Ego Network의 ROS 연결을 확인한다.
5. 해당 환경 manifest로 30개를 한 번에 실행한다.

학습 데이터는 `rolling-grouped`를 사용한다. 주행 중 6회 연속 trigger하고
마지막에 한 번만 파일 쓰기 완료를 기다린다. 그 뒤 MORAI 공통 timestamp로
Camera 4대·LiDAR·GT를 6개 프레임으로 묶고, 필수 파일이 빠진 묶음이
있으면 그 장면만 실패 처리한다. 원래 `rolling-fast`는 파일 묶음을
검증하지 않는 benchmark 모드이므로 학습 데이터에는 사용하지 않는다.

## 4. 환경별 결과 검사

각 실행이 끝나면 출력된 `ros_capture_poc_report.json`에서 다음을
확인한다.

```text
requested_scenes: 30
completed_scenes: 30
failed_scenes: 0
completed_captures: 180
```

실패 장면이 있으면 해당 report를 보존하고 실패 원인을 해결한 뒤 batch를
다시 실행한다. 완전한 report가 환경별로 하나씩 생긴 뒤 handoff index를
만든다.

```bash
cd ~/AjouNice2026

python3 dataset/morai_od/scripts/build_od180_handoff_index.py \
  --capture-root /home/luv/morai_cap/od180_v1
```

이 명령은 최신의 완전한 환경별 report 여섯 개를 선택하고, 1,080개 파일
묶음과 split을 검증한 뒤 `samples.jsonl`과 작은 metadata bundle을 만든다.

## 5. 학습 담당자에게 전달

`/home/luv/morai_cap/od180_v1` 전체를 전달한다. 학습 담당자는
`handoff_manifest.json`을 먼저 읽고 `samples.jsonl`을 dataset index로
사용한다.

파일명으로 split을 재추정하거나 frame 단위 random split을 하지 않는다.
반드시 `samples.jsonl`의 `split` 또는 다음 base split 파일을 사용한다.

- `handoff_metadata/dataset/morai_od/manifests/od180_train.txt`
- `handoff_metadata/dataset/morai_od/manifests/od180_val.txt`
- `handoff_metadata/dataset/morai_od/manifests/od180_test.txt`

학습 코드가 다른 디렉터리 구조를 요구하면 원본을 이동하지 말고
`samples.jsonl`을 읽어 symlink 또는 변환 데이터셋을 별도로 만든다.
