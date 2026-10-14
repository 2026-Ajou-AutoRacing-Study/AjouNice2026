# Validator 녹화·로그·공유 분석 계약 검토

확인 기준: 2026-10-14, 로컬 `dataset/base5`의 `17f23cfe` 소스.
팀장님 브랜치는 fetch 시점 `planning/autoware-universe-to-ros1`의 `3d85a22f`다.
문서 상태: 최초 요청의 이력. 아래의 누락 분석은 당시 소스에 대한 결과이며 현재 구현 상태가 아니다.
후속 validator 변경은 분석 모드의 진단 발행/녹화, 공유 묶음, 원격 파일 검증과 수동 공유 준비를 구현했다.
2026-10-14 전체 통합의 실행은 [현재 런북](../planning/SCENARIO_VALIDATOR_RUNBOOK.md#5-결과-확인공유)을 따른다.
실제 팀 PC의 녹화·공유 왕복 검증은 코드 통합과 별도로 확인한다.

## 현재 GUI에서 할 수 있는 것

녹화 모드와 공유 모드는 서로 독립이다. 녹화를 켜도 공유 모드가 `basic`이면 bag을 보내지 않는다.
토픽을 하나씩 선택할 필요는 없고 목적에 맞는 녹화 프리셋을 선택하면 된다.
`직접 선택`은 기존 YAML의 `selected_topics`가 프로파일을 대체하므로 추가 누락에 주의한다.

| 녹화 모드 | 현재 저장 범위 | 남는 제한 |
|---|---|---|
| 기본 녹화 | Ego/Object/Collision, localization·TF, 검출·측정·추적·예측, planner·제어, rosout 등 | 상세 frame timing·노드 진단 및 planner 시간 패턴은 자동 포함하지 않는다 |
| 지연 분석 (센서 제외) | 경량 결과·모델 status/latency, planner processing-time 패턴 | 원본 재추론 불가. 상세 frame timing과 후단 노드 진단은 아직 빠져 있다 |
| 인지 분석 (센서+영상) | 분석 결과, `/sensor/lidar1`, 카메라 4개 compressed 토픽, 화면 영상 | raw LiDAR packets가 없고 planner 시간 패턴도 이 모드에는 자동 추가되지 않는다 |

공유 GUI의 `bag`은 기본 자료+bag, `video`는 기본 자료+MP4, `full`은 기본 자료+bag+MP4다.
Drive 자동 업로드를 켜고 목적에 맞는 공유 모드를 선택해야 한다.
`full`이라는 이름은 campaign 상위 폴더 전체를 업로드한다는 뜻이 아니다.
분석용 full-course 예제도 `upload.enabled: false`, `profile: basic`이므로 기본값만으로 공유 완료를 기대하면 안 된다.

## 이미 저장하는 로그와 업로드에서 빠지는 자료

`runtime.Processes`는 실행한 프로세스 stdout/stderr와 명령을 campaign의
`stack_logs/*.log`, `stack_logs/commands.json`에 자동 저장한다.
회차 폴더에는 `input.json`, 실제 `scenario.json`, `events.jsonl`, 결과 JSON,
`recording_audit.json` 및 덤프에 성공한 `ros_parameters.yaml`이 저장된다.

`runner._upload_commands()`는 회차 폴더를 공유 모드에 따라 복사하고 campaign 상위의
`summary.csv`, `campaign_config.yaml`만 별도로 보낸다. `_upload()`는 `campaign_state.json`도 보낸다.
상위 `stack_logs/`, `model_runtime.json`, `model_launch.json`, `model_process_receipt.json`은 이 경로에서 보내지 않는다.
회차 JSON에 모델 정보와 일부 로그 신호가 있더라도 전체 서버·노드 로그와 실행 명령을 대체하지 못한다.
`/rosout` bag 역시 프로세스의 전체 stdout/stderr를 대체하지 못한다.

현재 GUI의 공유 설정은 이 업로드 경로를 선택한다. 별도의 campaign 전체 공유 기능으로 보완되는 경로는 확인되지 않았다.
따라서 **업로드된 자료만 받아서 상세 지연·설정·프로세스 실패까지 모두 분석하는 구성은 아직 완성되지 않았다.**

## 발행과 녹화를 함께 연결해야 하는 진단

- E25: `/perception/bevfusion/{status,latency_ms,frame_timing}`.
- validator Base5 E24/E9: `/perception/base5_trial/{status,latency_ms,frame_timing}`.
- 후단: `/map_based_prediction/diagnostics/processing_time_ms`,
  `/tracked_object_motion_policy/diagnostics/processing_time_ms`.

현재 로컬 토픽 계약에는 두 모델 namespace의 status/latency와 측정 객체가 포함되지만 `frame_timing`은 없다.
Base5 실행기의 `models.trial_commands()`는 `publish_frame_timing:=true`를 전달하지 않으며 launch 기본은 false다.
E25 실행기는 환경 변수 `BEVFUSION_PUBLISH_FRAME_TIMING`을 사용하지만 기본 false다.
후단은 `object_pipeline.launch publish_processing_diagnostics:=true`가 필요하며 현재 validator는 이를 전달하지 않는다.
토픽 이름만 녹화 목록에 넣어도 메시지가 발행되지 않으면 진단 자료는 생기지 않는다.

현재 `complete_for_latency`는 모델 status/latency/measurement와 요청한 planner 시간 존재를 확인한다.
상세 stage timing, 모든 관심 노드의 진단, CPU 부하, 공유 파일 완전성을 보증하는 지표는 아니다.

팀장님 브랜치의 확인 시점 `basic.json`은 Base5 trial telemetry와 measurement 토픽이 빠져 있다.
로컬의 토픽 계약·지연 프리셋·audit 보완을 먼저 반영해야 한다. 팀장님 실제 checkout과 빌드·설치본도 확인한다.

## Validator 담당 에이전트에게 전달할 요청

목표는 **팀원이 분석용 프리셋으로 실행하고 공유하면, 업로드된 묶음만 받아도 요청한 분석을 재현하는 것**이다.
개별 토픽 체크나 별도 터미널 녹화를 기본 절차로 요구하지 않는다.

1. 최신 모델 namespace·measurement·recording audit 변경을 planning 브랜치에 반영하고 소스와 설치본을 일치시킨다.
2. 분석 프리셋에서 `publish_frame_timing`과 `publish_processing_diagnostics`를 자동 활성화한다.
   모델별 실제 namespace의 frame timing, 위 두 후단 노드 진단, planner 시간 패턴을
   지연 분석뿐 아니라 인지 분석에도 연결한다. 평가 신호를 detector 입력이나 동작 정책으로 사용하지 않는다.
3. 공유 묶음에 case 자료 외에도 campaign 로그·실행 명령·실제 모델 receipt/config를 포함한다.
   실행 종료 후 최종 로그를 전송하고, 중간 종료·다중 회차·재시도에도 기존 증거를 보존한다.
   토큰·rclone 자격증명·환경 변수 전체 덤프는 공유하지 않는다.
4. 분석 프리셋 선택 시 bag을 제외하는 공유 설정이면 경고하고 `bag`/`full`을 제안한다.
   고용량 센서·영상·packets는 별도 선택과 용량 안내를 유지한다.
   raw `/sensor/lidar1/packets`는 50/51·cut-angle 재구성 시험에만 필요하므로 별도 패킷 분석 옵션을 둔다.
5. `주행 성공`, `녹화 완료`, `결과 분석 가능`, `상세 지연 분석 가능`, `재추론 가능`,
   `패킷 재구성 가능`, `공유 완료`를 구분한다. 실제 메시지 수·시간 범위와 업로드 파일 목록/크기/SHA를 검사한다.
   없는 토픽의 요청만으로 성공을 표시하지 않는다. 센서 age도 ROS header→수신, host wall time,
   GPU/server 비용을 구분해 기록한다.
6. 회차 provenance에 simulator MR/MRP, localization 보정 여부/offset,
   실제 pose source·frame·시간 정책·센서/네트워크 SHA를 저장한다.
   [GT 좌표 계약](GT_EVALUATION_COORDINATE_CONTRACT.md)을 따라 보정된 localization을 다시 보정하지 않는다.

검증은 E24/E25 각각의 GUI 프리셋→발행 인자→실제 bag→공유 묶음 재다운로드까지 이어서 수행한다.
미발행 진단, raw 센서 제외, bag 제외 공유, 업로드 실패, 중단 후 재공유, 민감 정보 미포함도 검사한다.
`rosbag info`만으로 detector 정확도나 planner가 실제 소비한 모든 prediction hypothesis를 인증하지 않는다.
그 분석에는 독립 GT와 planner 측의 실제 소비 후보 기록이 별도로 필요하다.

## 이번 점검의 검증 범위

설치 workspace 환경에서 기존 `test_latency_contract.py` 10개와
`test_recording_options.py` 17개, `test_campaign.py` 29개가 통과했다(총 56개).
이 테스트들은 현재 GUI/기본 녹화·업로드 계약의 회귀 검사이며,
위의 새 공유·상세 진단 요청이 구현됐다는 근거가 아니다.
실제 팀장님 PC 녹화·Drive 재다운로드 검증은 담당 구현 후 수행한다.
