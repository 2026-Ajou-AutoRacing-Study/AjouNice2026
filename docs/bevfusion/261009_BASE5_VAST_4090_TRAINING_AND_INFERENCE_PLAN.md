# Base5 Vast.ai 4090 학습과 추론 시험 계획

2026-10-09에 작성하고 2026-10-10에 진행 상황을 갱신했다. Sunny13 독립 5K를 먼저
학습하고, 이후 나머지 5개 환경에서 새로 배치한 25K를 더해 6개 환경의 30K로 확장한다.
[환경별 새 공간 배치 합의](../../dataset/morai_od/docs/261009_base5_remaining_environments_spatial_diversity_plan.md)가
과거의 동일 배치 반복 계획보다 우선한다. 첫 임대는 단일 RTX 4090의 On-demand 방식을 권장한다.
로컬 5060Ti에서 입력과 pooling의 정합성 및 연산 비용을 확인한 뒤, 원격 4090에서
다시 측정하고 학습한다. 아래의 10-10 진행 기록이 10-09의 초기 계획보다 우선한다.
사용자가 임대를 완료했고 2026-10-10에 Direct SSH 접속과 실제 CUDA 연산을 확인했다.
이 agent는 임대·결제·이미지 게시를 수행하지 않았다. 이후 전송·설치·검증 상태는 아래 원격 진행 기록을 따른다.
기존 수집 데이터, production E25 모델, tracker는 변경하지 않는다.

**최신 상태 — 2026-10-10 21시:** Sunny13 5K의 전달·전수 검증 후, 300장 비용 비교,
64장 동등성, 실제 train 로더의 batch 1/2/4/8, 745장 전체 validation과 원격 저장·재개를
통과했다. 512×1056·batch 4를 추천하며 본학습은 시작하지 않았다.
**실제 외부 백업만 네트워크 문제로 미완료**다. 원격 Google API DNS 요청과 일부 직접
TLS 연결이 timeout이었다. 큰 파일을 반복 전송하거나 인스턴스를 변경하지 않는다.
[확대 표본·실제 로더·시간·남은 gate](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#6-4090-확대-표본학습-전-검증--2026-10-10-2023-이후)가 아래 과거 진행 기록보다 우선한다.

## 2026-10-10 원격 검증 진행 — 본학습 시작 전

### 이미지 증강 선택 — 2026-10-10 23:05 KST 로컬 검토

개발용 rosbag 12개의 1,536장 모두 JPEG Q90·4:2:0 양자화 표를 확인했다.
Base5 native PNG 168장면·672개 이미지와 비교해 **JPEG만 학습에서 카메라별 80% 확률로
적용하고 blur·노출·noise는 OFF**로 결정했다. 압축은 원본 카메라 해상도에서 RGB/BGR
변환을 명시해 수행하고 이후 기존 resize/crop을 적용한다. val/test·추론·원본 GT는 그대로다.
[측정·CPU 검사·한계](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#image-domain-decision-261010)를
따르며 AP 개선이 증명된 것은 아니다.

본학습 후보는 `perception/detection3d/bevfusion/configs/bevfusion_base5_dense_512_jpeg90_train.py`다.
512×1056·batch 4·±100×30m는 유지한다. all-OFF 비용 config는 과거 재현용으로 보존한다.
사용자가 Vast 재개를 요청하면 새 소스/config 전달과 새 준비 경로·해시 계약으로 확정한다.
기존 all-OFF 준비 계약을 재사용해 resume하지 않는다. **종전 약 4.8시간은 JPEG 증강 이전
비용**이며, 짧은 train step에서 worker/prefetch 비용을 확인해 갱신한다.
이번에는 원격 접속·인스턴스 재개·GPU 실행을 하지 않았으며 외부 백업 gate도 미해결이다.

### 좌우 범위 선택 확정 — 2026-10-10 22:43 KST

이번 Base5 학습은 **전후방 ±100m·좌우 ±30m**를 유지한다. 이미지 512×1056·batch 4
추천은 그대로다. ±40m는 비용 비교를 마쳤지만 미션 필요성이 충분히 입증되지 않아
이번 학습에서 기각했다. 평균 추론 비용이 불가능해서 기각한 것이 아니며,
기존 5K에 신규 띠의 중심 GT가 없다는 점과 핵심 미션의 데이터·검증 예산을 고려했다.
[원본 5K·합류/로터리 경로 확인과 남는 한계](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#lateral-range-decision-261010)를
현재 범위 계약으로 사용한다. 비용 전용 lateral40 config를 본학습에 자동 선택하지 않는다.

사용자는 로컬 검토를 위해 Vast 인스턴스를 중단했다고 보고했다. 이번 문서화에서는 원격
접속·재개·학습을 수행하지 않았다. 이후 사용자 요청으로 재개할 때 기존 ±30m 설정과
외부 checkpoint 백업 gate를 확인하며, 범위 확대용 데이터 생성·5K 재수집은 하지 않는다.

### 확대 검증 완료와 네트워크 gate — 2026-10-10 21시

원격 입력은 `/root/base5_sunny13_portable_261010`, 결과는
`/root/base5_qualification_261010_2023`이다. 로컬 근거는
`artifacts/evaluation/bevfusion/base5_vast_qualification_261010/`에 회수했다.
본학습·다른 환경 수집·production 승격은 수행하지 않았다.

- 512의 일반 200장 서비스 평균/p95는 69.96/70.33ms다. 드문 최대 175ms 지연이 있어
  ROS 풀스택의 모든 프레임 13Hz 보장은 아니다. 단계별 계측 결과와 별도 재측정을 구분한다.
- 실제 train batch 4는 609.57ms/step, 정상 최대 할당 20.64GiB다. batch 8은
  40.67GiB를 사용하면서 처리량은 약 2.7%만 높아 batch 4를 추천한다.
- validation 745장의 실제 처리·4클래스 metric 실행을 통과했다. 현재 validation은 FP32다.
  4-iteration smoke의 AP를 새 모델의 성능으로 해석하지 않는다.
- 제안 512/batch 4/24 epoch는 train+val 약 4.8시간에 저장·장기 부하 여유를 더한다.
  카메라 backbone/neck만 감사한 nuScenes 가중치로 초기화하고 나머지는 새로 학습한다.
  24 epoch는 임시 5K 제안이며 최적 epoch가 증명된 것은 아니다.
- 실제 checkpoint 493,471,045 bytes의 외부 업로드는 실패했다. 로컬의 OAuth scope,
  Drive 여유와 폴더 쓰기 권한은 정상이며 원격의 DNS/외부 연결 문제를 관측했다.
  copy 실패 전후 원격 체크포인트와 원본 5K는 보존했다.
- 새 로컬 토큰 갱신 watcher는 큰 파일 전송 전에 최대 15초의 읽기 전용 preflight를 수행한다.
  이번 실제 DNS 실패에서 큰 업로드를 시작하지 않고 종료하는 것을 확인했다.
  watcher 구현과 실제 외부 백업 완료를 혼동하지 않는다.

다음은 **Vast 외부 연결 복구 → 전체 checkpoint 업로드·재읽기 SHA256 → 반복 watcher 확인 →
최종 코드/config 해시로 준비 재생성 → 사용자 승인 후 본학습**이다.
SSH 대용량 회수도 짧은 검사에서 느렸고 Proxy banner가 timeout이어서 자동으로 전환하지 않았다.
새 인스턴스 임대·재시작·stop/destroy·Cloud Sync는 사용자 선택이 필요하며 실행하지 않았다.

네트워크가 복구된 뒤 로컬에서 실행할 watcher 형태는 다음과 같다. 실행 디렉터리는
본학습을 실제로 준비한 최종 경로로 지정한다. 로컬 PC·watcher가 켜져 있어야 한다.
이 명령은 현재 네트워크에서 성공한 백업 명령이 아니라 **후속 검증용**이다.

```bash
python3 -u perception/detection3d/bevfusion/tools/backup_base5_vast_access_only.py \
  --host root@175.155.64.187 --port 19958 \
  --work-dir /root/base5_training_sunny13_512_b4_e24_261010_v2 \
  --destination backup:model/bevfusion/base5/checkpoint_backups \
  --root-folder-id 12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP \
  --watch --interval-sec 120 \
  --log artifacts/evaluation/bevfusion/base5_vast_qualification_261010/training_backup.jsonl
```

epoch 경계의 원자적 full checkpoint, hash-addressed payload와 SHA 재읽기, 메타데이터,
마지막 `LATEST` pointer 순서를 유지한다. 접근 토큰만 원격 anonymous RAM에 두고
갱신 토큰/client secret은 로컬에 남긴다. 실패 시 원격 성공분을 삭제하거나 미완료본을 latest로 처리하지 않는다.

사용자는 계획의 환경 설치부터 초기화·저장/재개·외부 백업 검증까지 수행하도록 승인했다.
임시 5K 본학습은 이번 범위가 아니다. 실제 GT로 수행하는 유한한 비용·복구 smoke와 본학습을 구분한다.
문서와 검증 근거는 로컬 AjouNice2026 저장소에 보존한다.

- 접속 확인: RTX 4090 한 장, 표시 메모리 49140MiB, driver 570.144,
  PyTorch 2.7.1+cu128 / CUDA 12.8, nvcc 12.8.61이다. 작은 CUDA 행렬 연산을 통과했다.
- 컨테이너 디스크는 200GB, 공유 메모리는 약 47GB다. MMCV·MMDetection3D·spconv·OpenCV 등
  BEVFusion 의존성은 처음 접속할 때 설치되지 않았다.
- 템플릿의 SSH는 tmux에 자동 연결된다. `Ctrl+B` 후 `D`로 분리하면 SSH도 종료되지만
  tmux 안의 작업은 유지된다. 현재 tmux의 셸에서 `exit`를 실행하는 것과 구분한다.
- Direct 업로드는 짧은 관측에서 수십 KB/s까지 느려졌다. 동일 머신의 Proxy SSH로 코드
  10,261,666 bytes를 약 1.4MB/s에 전달하고 SHA256을 검증했다. 접속 가능성과 전송 성능은 별개다.
- 16:52 KST에 별도 `ajou_setup` tmux에서 직접 설치를 시작했다. 로그와 종료 코드는
  `/root/ajou_setup_logs/install.log`, `/root/ajou_setup_logs/install.exit`에 남긴다.
- 데이터는 기존 Drive 백업과 동일한 무손실 19조각의 Proxy SSH 병렬 전송을 시험했지만,
  지속 전송 속도가 낮아 중단했다. 남은 대용량 전송 프로세스가 없음을 확인했다.
  부분 파일을 완료 데이터로 사용하지 않으며, 원본과 검증된 Drive 백업은 그대로 유지한다.
  작은 벤치마크 묶음과 초기 가중치는 전송을 완료했다. 데이터 전체 전송 완료와 구분한다.
- 사용자의 추가 지침에 따라 대용량 전송은 짧은 속도 측정 후 판단한다. 초기 순간 속도가
  아닌 지속 실효속도로 남은 시간을 계산하고, 예상이 1시간 이상이거나 속도가 불안정하면
  본 전송을 시작하지 않는다. 진행 중 같은 문제가 확인되면 중단하고 사용자에게 알린다.
  느린 경로를 반복해서 시도하지 않으며, 더 빠른 경로가 확인되기 전까지 5K 전송은 보류한다.
  전송·조각 및 전체 압축본 해시·해제 후 파일별 SHA256 검증 완료 전에는 학습 입력으로 사용하지 않는다.
- Drive OAuth/refresh token은 임대 머신에 복사하지 않는다. 외부 체크포인트 백업은
  원격 완료 파일을 SSH로 로컬에 회수한 뒤 기존 로컬 rclone으로 업로드·재읽기 검증하는 경로를 우선한다.
  로컬 PC와 연결이 유지돼야 하며, 원격 자체의 독립 Drive 백업이 검증됐다고 표현하지 않는다.

2026-10-10 17:17 KST까지 직접 설치를 완료했고, torch 2.7.1+cu128 / MMCV 2.1.0 /
MMEngine 0.10.7, CUDA rotated NMS 실행과 BEVFusion 확장 import를 확인했다. 설치 종료 코드는
0이다. 전체 모델·분리 pooling 검증, 4090 비용, 초기화·재개·외부 백업 검증 완료와 구분한다.

### Drive 직접 다운로드 검사 — 2026-10-10 17:18 KST

사용자는 실제 rclone 병렬 다운로드를 승인했고, 데이터 전송 실효속도가 10MB/s 이하면
중단하도록 지시했다. 본학습이나 5K 전체 전송을 시작한 검사가 아니다.

| 검사 | 방법 | 결과와 해석 |
|---|---|---|
| 17:05 단일 연결 검사 | Python Drive API Range, 약 5초 | 약 1.11MB / 0.222MB/s다. 연결 준비 시간을 포함한 짧은 표본이며 rclone 전송 검사가 아니다. 서버의 지속 회선 속도로 단정하지 않는다. |
| 17:18 실제 rclone 검사 | rclone 1.75.2, 파일 4개 병렬, 파일당 단일 스트림 | 약 124.76MB를 전송했다. 첫 데이터 관측까지 4.07초, 관측 시작 이후 평균 13.848MB/s, 마지막 초별 구간은 약 18~24MB/s다. |

실제 rclone 검사는 시작부터 종료까지 13.13초이며, 준비 시간을 포함한 전체 평균은
약 9.5MB/s다. 앞선 합의대로 준비 시간과 데이터 전송 구간을 분리해 기록했다.
초기 연결 이후 속도가 상승했으며, 데이터 전송 구간 평균은 10MB/s 기준을 넘었다.
전체 19.44GB 압축본의 지속 속도나 완료 시간을 보장하지 않는다. 관측 평균이 유지된다면
순수 전송은 약 23분이지만 장시간 안정성·해제·무결성 검사는 별도다.

검사만 수행한 뒤 rclone에 SIGINT를 보내 종료했으므로 종료 코드 130은 의도된 중단이다.
검사용 임시 파일을 제거했으며 5K 전체 전송은 시작하지 않았다. 접근 토큰만 암호화된
SSH 입력과 메모리 pipe로 전달했고, 원본 인증 파일·갱신 토큰을 원격에 복사하지 않았다.
검사 도구의 공식 ZIP SHA256은 기존 로컬 검증본과 동일하다.
상세 근거는 Git에서 제외한 `artifacts/handoff/base5_drive_speed_probe_261010/parallel_result.json`이다.

### 전송 방법 추가 비교 — 2026-10-10 17:24 KST

사용자는 다른 방법도 각각 짧게 비교하도록 승인했다. 설치 다운로드가 끝나고 다른
전송 프로세스가 없는 상태에서 검사했다. 데이터 전송 구간이 10MB/s 이하면 중단하며,
전체 5K 전송은 시작하지 않았다. 각 방법은 최대 22~30초, 제한된 원본 압축 조각으로 검사했다.

| 방법 | 결과 | 판단 |
|---|---|---|
| Drive → 서버, curl 단일 연결 | 준비 제외 약 3.529MB/s, 준비 포함 약 2.279MB/s | 기준 미달로 중단했다. |
| Drive → 서버, curl 4개 Range 연결 | 준비 제외 약 11.650MB/s, 준비 포함 약 8.119MB/s; 64MiB 전송을 완료했고 모든 응답은 HTTP 206이다. | 병렬 Drive 경로가 rclone 외의 도구에서도 유효함을 확인했다. |
| 로컬 → 서버, Direct rsync | 약 21.57초 동안 검사 파일에 262144 bytes만 기록되고 정체됐다. | 중단했다. 이 파일 크기 관측만으로 정확한 네트워크 수신량이나 0MB/s라고 단정하지 않는다. |
| 로컬 → 서버, Direct scp | 준비 제외 약 0.692MB/s, 준비 포함 약 0.571MB/s | 기준 미달로 중단했다. |
| Drive → 서버, rclone 4개 파일 재검사 | 403 `rateLimitExceeded`; 전송 0 bytes | 속도 측정 전에 API 호출 제한으로 실패했다. |
| Drive → 서버, rclone 8개 파일 검사 | 같은 403 `rateLimitExceeded`; 전송 0 bytes | 병렬 수 확대의 효과는 확인하지 못했다. 자동 재시도하지 않았다. |

curl 비교는 같은 Drive 백업 조각의 제한된 범위를 내려받았고, SSH 비교는 로컬에 보존된
마지막 압축 조각을 업로드했다. 출발지와 측정 시점이 다르므로 도구 자체의 우열을 판정하는
통제 실험이 아니다. 다만 현재 환경에서 Drive 병렬 다운로드가 Direct 업로드보다
유리한 경로라는 근거를 추가했다. 짧은 표본의 초기 준비·TCP 상승 구간과 지속 속도를 구분한다.

기존 rclone 4병렬의 13.848MB/s 기록은 유효하지만, 최신 재검사는 API 제한에 막혔다.
현재 로컬 `ajou_drive`는 공유 client_id를 사용한다. 전용 client_id가 API 호출 제한
완화에 도움이 될 수 있으나 이번 오류의 정확한 제한 범위나 해결을 보장하지 않는다.
계정 연결·OAuth 앱 생성·공유 권한 변경은 이번 검사에서 수행하지 않았다.
[rclone 공식 client_id 설명](https://rclone.org/drive/#making-your-own-client-id)을 참고한다.
curl 병렬은 대안이나 장시간 조각 관리·무결성·재개 경로를 검증한 완성 수집 전송기로 간주하지 않는다.

검사 파일은 각 임시 디렉터리에서 제거했고 원본과 Drive 백업은 변경하지 않았다.
근거는 Git에서 제외한 `artifacts/handoff/base5_drive_speed_probe_261010/`의
`method_suite_result.json`, `parallel_4_recheck_result.json`, `parallel_8_result.json`이다.

### 호출 제한 완화 재검사 — 2026-10-10 17:30~17:32 KST

로컬에서는 기존 백업 폴더의 목록을 읽었으며 압축 조각 19개와 관리 파일 6개를
확인했다. 원격 rclone은 4개 파일 병렬을 유지하되 API 요청을 초당 1회로 제한하고
불필요한 재시도를 막아 재검사했다. 결과는 다시 403 `rateLimitExceeded`, 전송량
0 bytes다. 로컬 목록 조회 성공만으로 원격 제한이 해소됐다고 판단하지 않는다.
[Google의 오류 설명](https://developers.google.com/workspace/drive/api/guides/handle-errors#ratelimitexceeded)에
따르면 이 오류는 프로젝트 요청 제한이며, 정확한 할당·호스트 영향은 이번 검사로 분리하지 못했다.

기존 접근 토큰으로 curl 4개 Range 연결도 한 차례 재검사했다. 준비 시간 2.506초,
전체 7.56초 동안 약 30.77MB를 받았고 데이터 전송 구간 평균은 6.088MB/s다.
10MB/s 기준 미달로 즉시 중단하고 임시 파일을 제거했다. 앞선 짧은 11.650MB/s
관측은 유지하되 지속 속도를 보장하는 근거로 사용하지 않는다. 전체 5K 전송은
시작하지 않았으며 느린 전송을 반복하지 않는다. 계정·공유 설정과 원본은 변경하지 않았다.

근거는 `artifacts/handoff/base5_drive_speed_probe_261010/parallel_4_paced_result.json`,
`continuation_result.json`이다. 기존에 전달하고 SHA256을 확인한 14장짜리 묶음은
원격에서 추가로 내부 파일 체크섬을 통과했다. 별도 `ajou_bench` tmux에서 추론 비용·pooling
동등성·유한한 실제 GT optimizer smoke를 시작했다. 전체 5K 전송이나 본학습이 아니다.
후속 검사도 17:34 KST에 완료했다. runner 3개 exit 0, CPU/CUDA 단위 검사,
비용 9개 case·실제 14장 동등성 6개 case·3-step 역전파 12개 case가 통과했다.
분리 B의 모델/서비스 평균은 448에서 48.84/65.79ms, 512에서 51.50/69.71ms다.
결과 ZIP을 로컬로 회수하고 원격 SHA256과 대조했다.
[원격 실측 표와 한계](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#5-vast-4090-실제-검증--2026-10-10-17311734-kst)를
따른다. 본학습·최종 입력 확정·원격 checkpoint 재개/외부 백업은 아직이다.

### 전용 Drive 앱 인증과 속도 재검사 — 2026-10-10 17:57 KST

사용자가 Google Cloud에서 Desktop OAuth 클라이언트를 생성하고 로컬 브라우저에서
기존 Drive 계정으로 로그인·권한 동의를 완료했다. 클라이언트 JSON의 구조·Google 인증
endpoint를 확인하고, 기존 `ajou_drive`와 별도의 `ajou_drive_private`를 구성했다.
기존 연결의 모든 설정값이 인증 전 백업과 동일함을 대조했다. 로컬 설정·JSON·백업의
파일 권한은 0600이며 인증 정보는 Git·문서·Vast에 복사하지 않았다. 원격 속도 검사에는
접근 토큰과 공개 client_id만 암호화된 SSH 입력·메모리 pipe로 전달했다.

전용 연결에서 백업 폴더 목록 25개(압축 조각 19개·관리 파일 6개)를 읽었다.
원격 rclone 4개 파일 병렬·파일당 단일 스트림·API 초당 1회 검사에서도 403이
발생하지 않았고 실제 데이터 다운로드를 시작했다. 이번 검사 범위에서 API 호출 제한이
재발하지 않았다는 뜻이지, 앞으로의 모든 호출이나 지속 속도를 보장하는 결과는 아니다.

첫 데이터 관측까지 3.07초, 전체 9.085초, 약 12.57MB 수신이며 데이터 전송 구간 평균은
2.073MB/s다. 사용자 기준인 10MB/s 이하여서 즉시 중단하고 검사 파일을 제거했다.
종료 코드 130은 의도한 SIGINT다. 전체 5K 전송·본학습은 시작하지 않았다.
전용 앱 설정 실패와 낮은 네트워크 실효속도를 구분한다. 자동으로 느린 전송을 재개하지 않는다.

이전 백업 업로드 로그의 상세 오류에는 `defaultPerMinutePerProject`,
`quota_limit_value=840000`, `quota_unit=1/min/{project}`가 명시돼 있다.
해당 이력은 공용 앱 프로젝트의 분당 요청 한도 초과로 확인되며, 우리 검사 몇 번만으로
이 요청량을 발생시켰다고 판단하지 않는다. 현재의 느린 다운로드는 그 403과 별개다.

근거는 Git에서 제외한 `artifacts/handoff/base5_drive_speed_probe_261010/private_parallel_4_result.json`이다.
클라이언트 secret·접근/갱신 토큰은 이 문서와 검증 산출물에 기록하지 않는다.
다음 단계는 이 인스턴스에서 다른 병렬 구성의 짧은 재검사를 할지, 다른 호스트를 선택할지
사용자와 결정하는 것이다. 인스턴스 임대 변경·중단·삭제는 자동으로 수행하지 않는다.

### 전용 연결 8병렬 검사 — 2026-10-10 18:04 KST

사용자의 요청으로 같은 전용 앱·백업에서 파일 8개 병렬, 파일당 단일 스트림,
API 초당 1회, 최대 30초/256MiB의 제한 검사를 수행했다. 첫 데이터까지 3.077초,
전체 9.109초 동안 46,379,008 bytes를 수신했고, 첫 데이터 관측 이후 구간 평균은
7.720MB/s다. 403 등 기록된 API 오류는 없었다. 10MB/s 기준 미달로 SIGINT를
보내 종료했으며 종료 코드 130은 의도된 중단이다. 임시 파일을 제거하고 전체 5K
전송은 시작하지 않았다. 17:57의 4병렬 표본 2.073MB/s보다 높은 결과지만 측정
시점이 달라 병렬화의 인과적 개선량이나 지속 속도로 단정하지 않는다.

이 평균이 유지된다면 19.44GB의 순수 전송은 약 42분이다. 연결 변동·검증·압축 해제
시간은 포함하지 않으며, 사용자 속도 기준을 임의로 완화하는 근거로 사용하지 않는다.
근거는 `artifacts/handoff/base5_drive_speed_probe_261010/private_parallel_8_result.json`이다.
이후 호스트 변경 또는 전송 허용 기준 변경은 사용자와 결정한다.

### 두 번째 호스트의 제한 검사 — 2026-10-10 18:17 KST

사용자가 기존 인스턴스 55165763을 유지하고 인스턴스 55178510을 추가 임대했다.
새 호스트는 124072, machine 38442이며 Direct SSH 접속과 RTX 4090의 49140MiB를
확인했다. 사용자 캡처에는 GPU 100%가 표시됐지만 접속 당시에는 0%, 1MiB, 30°C였다.
이 순간 관측만으로 다른 사용자의 GPU 점유나 지속 경합을 단정하지 않는다.

새 호스트의 rclone 공식 ZIP 다운로드는 20초 제한에 약 9.21MB만 수신해 중단됐다.
로컬의 검증된 ZIP을 Direct scp로 보내는 검사도 20초 제한에서 종료됐다. 따라서 새
호스트에 rclone 설치를 완료하지 못했으며, 기존 8병렬 rclone과 동일한 재검사를 했다고
기록하지 않는다. 기존 curl 도구로 전용 Drive 앱의 같은 백업 조각에 4개 Range 연결을
사용한 제한 검사를 수행했다. 인증 토큰은 메모리와 SSH 입력에만 두었다.

| 대상 | 조건 | 결과 |
|---|---|---|
| 기존 55165763, 18:04 기록 | 전용 앱, rclone 파일 8개 병렬 | 데이터 구간 평균 7.720MB/s다. 당시 기준 미달로 중단했다. |
| 새 55178510, 18:17 검사 | 전용 앱, curl Range 4개 병렬 | 준비 2.012초, 전체 7.037초에 3,768,320 bytes를 수신했다. 데이터 구간 평균 0.748MB/s, 준비 포함 0.536MB/s다. 기준 미달로 중단했다. |

서로 다른 도구·병렬 조건·측정 시점이므로 호스트 속도 배수를 산출하지 않는다. 원래
호스트에는 비교 직전 사용자의 다운로드가 시작돼 추가 검사를 건너뛰고 작업을 보존했다.
새 호스트가 더 빠른 경로라는 근거는 확보하지 못했다. 두 IP가 같은 대역이라는 점만으로
같은 회선이라고 단정하지 않으며, 양쪽이 느리다는 사실도 우리 접근 방식의 오류를 증명하지 않는다.
Drive 다운로드 데이터는 서버가 직접 수신하므로 `ssh6.vast.ai` 프록시를 경유하지 않는다.

전체 5K 전송·설치·GPU 실험은 새 호스트에서 시작하지 않았다. 검사 임시 파일과 도구
불완전 ZIP을 제거하며, 기존 인스턴스와 Drive 원본은 변경하지 않는다. 근거는 Git에서
제외한 `artifacts/handoff/base5_drive_speed_probe_261010/two_host_private_https_result.json`이다.
임대 종료·삭제는 사용자가 결정한다.

이후 기존 인스턴스의 사용자 rclone 프로세스를 읽기 전용으로 관측했다. 5.001초 동안
다운로드 폴더의 파일 논리 크기 합계가 2,539,630,287에서 2,572,136,143 bytes로
증가했으며 파일 증가 속도는 6.500MB/s다. 실제 네트워크 전송률이나 장기 평균과 구분한다.
`du -sh`의 8.1GiB는 디스크 할당량이며 완료 다운로드 크기로 사용하지 않는다.
현재 증가 속도가 유지된다면 전체 19.44GB에서 남은 순수 전송은 약 43분이지만
이 짧은 관측으로 완료 시간을 보장하지 않는다. 사용자 수동 전송은 중단하지 않았다.

### 수동 전송 중단 원인과 부족 조각 복구 — 2026-10-10 19:13~19:32 KST

사용자가 전달한 종료 로그에서 `--max-duration 60m --cutoff-mode HARD`의 제한에
도달한 것을 확인했다. 19:13:02 KST에 `context deadline exceeded`와
`max transfer duration reached as set by --max-duration`가 기록됐다. 이 종료 원인을
토큰 만료나 Drive API 제한으로 해석하지 않는다. 초기에 실패한 `part001`의 구체적인
오류는 전달된 로그에 없어 확정하지 않는다.

원격에는 완성된 17개 조각과 메타데이터가 남았다. 이 17개는 SHA256을 모두 통과했으며
`part001`과 `part017`만 없었다. 약 90%까지 진행한 `part017`의 임시 파일은 rclone이
종료 처리하면서 삭제했다. 마지막 `100%`·`23/23` 표시는 실패 파일이 분모에서 빠진
통계이므로 전체 백업 완료 판정으로 사용하지 않는다.

사용자 승인 후 19:29 KST부터 두 조각만 다시 다운로드한다. 갱신 토큰과 client secret은
로컬에 유지하고 새 접근 토큰만 SSH 입력과 원격의 익명 메모리 설정으로 전달한다.
기존 검증된 파일은 다시 받거나 삭제하지 않는다. 파일 2개 병렬·파일별 stream 4개,
초당 요청 1개 제한, 제한적인 재시도를 사용한다. 35분 `SOFT` 제한은 새 전송 시작을
막되 진행 중인 전송을 끝내도록 설정한 것이며, 35분 안에 완료된다는 보장은 아니다.
[rclone 시간 제한](https://rclone.org/docs/#max-duration-duration)과
[멀티스레드 전송](https://rclone.org/docs/#multi-thread-streams-int)을 따른다.

19:32 KST의 초기 120초 관측에서 약 220MB를 수신했고 오류는 0개다. 표시 속도는
약 2.44MB/s이며 지속 속도나 완료 시간을 보장하지 않는다. 당시에는 두 조각의 완료·전체
압축본 검증·해제 완료가 미확인이었다. 완료 판정은 19개 조각 SHA256 → 결합 압축본
SHA256 → 해제 파일 전수 SHA256 및 infos 참조 경로 검사를 모두 통과해야 한다.
본학습은 시작하지 않는다.

민감 정보 없는 복구 로그와 실행기는 Git 제외 경로
`artifacts/handoff/base5_drive_speed_probe_261010/`에 보관한다.

### 부족 조각 복구와 전체 데이터 검증 완료 — 2026-10-10 19:49 KST

두 조각의 복구는 1054.72초, 약 17분 35초에 정상 종료했다. 추가 전송은
2,147,483,648 bytes이며 오류는 0개다. 복구 전송 전체 평균은 약 2.04MB/s다.
초기 병렬 속도와 마지막 연결 속도가 다르므로 이를 호스트의 고정 성능으로 해석하지 않는다.
완성된 기존 17개 조각은 그대로 재사용했다.

이어 수행한 검증·압축 해제는 117.81초에 완료했다. 19개 조각 SHA256과 결합 압축본
SHA256이 모두 로컬 원본과 일치했다. 결합 압축본의 SHA256은
`ab24307ee17007c4170dacabb00e6c97130e090ddbdb45f0e99c38b244a8374f`이다.
별도 결합 압축 파일을 만들지 않고 스트림으로 해제했으며 원격의 다운로드 조각도 보존했다.

| 확인 항목 | 실제 원격 결과 |
|---|---|
| 해제 경로 | `/root/base5_sunny13_portable_261010`이다. |
| 전수 검증 | 173,290개 파일의 크기·SHA256과 infos 참조 경로 검사를 통과했다. |
| 장면·분할 | 5,000장, train 4,255 / val 745 / synthetic test 0장이다. mission test bag과 별개다. |
| 입력 후보 | `448x896`·`512x1056` 모두 같은 train/val 분할을 유지한다. |
| 클래스 | `vehicle / pedestrian / obstacle / animal` 4개다. |
| 분할 누출 검사 | 두 후보 모두 train/val의 token 겹침과 scene group 겹침이 0개다. |
| 해제 payload | 26,591,499,150 bytes다. 조각과 해제 데이터를 함께 보존한 뒤 여유는 약 154.16GiB다. |
| 학습 상태 | 본학습은 미시작이다. production E25와 원본 수집 데이터는 불변이다. |

근거는 위 Git 제외 경로의 `recovery_missing_parts_261010.jsonl`,
`verified_restore_261010.jsonl`, `restored_infos_check_261010.json`이다.
갱신 토큰·client secret은 임대 머신에 저장하지 않았다. Cloud Sync도 실행하지 않았다.
다음은 최종 입력 선택과 실제 5K loader·짧은 저장/재개·체크포인트 외부 백업 검증이다.
이번 데이터 무결성 통과만으로 모델 정확도나 장기 학습 재개까지 검증됐다고 표현하지 않는다.

### 현재 전송 유지와 Cloud Sync 후속 선택 — 2026-10-10 합의

사용자는 두 번째 인스턴스 55178510을 destroy했다고 보고했다. 이 agent가 종료·삭제한
것은 아니다. 기존 인스턴스 55165763에서 사용자가 실행한 rclone 다운로드를 유지하고
먼저 마무리한다. 이 합의 뒤 수동 전송은 시간 제한으로 종료됐으며, 최신 상태는 위의
부족 조각 복구 완료 기록을 따른다. 이 합의 시점에는 전체 다운로드·압축 해제 완료가 미확인이었다.
Cloud Sync 전환을 위해 현재 전송을 끊거나 인스턴스를 정지하지 않는다.

현재 데이터 경로는 **Google Drive → Vast 서버**다. 로컬 PC의 SSH는 실행 지시와
진행 표시를 전달하며 대용량 다운로드가 집 Wi-Fi나 SSH 프록시를 경유하지 않는다.
기존 rclone은 컨테이너 내부에서 실행되므로 인스턴스가 실행 중이어야 한다.
표시된 회선 속도는 Drive와의 지속 전송 속도를 보장하지 않는다. 이번 검사만으로
병목을 특정 호스트·Drive·공통 회선 또는 도구 설정 하나로 확정하지 않는다.

Vast Cloud Sync는 향후 30K 등 대용량 전송 때 필요하면 검토하는 **미실행 선택지**다.
Docker 인스턴스에서 Vast 측 호스트가 클라우드 저장소와 데이터를 복사하므로 정지
상태에서도 전송할 수 있다. 핵심 이점은 다운로드 대기 중 GPU 가동 비용을 줄이는
것이며 rclone보다 빠르다는 보장은 없다. 저장 공간·대역폭 비용은 별도로 남는다.
[공식 Cloud Sync](https://docs.vast.ai/guides/instances/storage/cloud-sync),
[정지 상태 데이터 전송](https://docs.vast.ai/guides/instances/storage/data-movement),
[요금 구성](https://docs.vast.ai/guides/instances/pricing)을 따른다.

#### 초기 준비의 역할 분담 — 아직 수행하지 않았다

| 항목 | 담당과 상태 |
|---|---|
| Drive 접근 계정 선택 | 사용자가 선택한다. 데이터 폴더만 접근 가능한 전용 Google 계정을 권장한다. 개인 전체 계정을 기본 연결하지 않는다. |
| Vast의 Google Drive 연결 | 사용자가 Settings → Cloud Connections에서 로그인·OAuth 동의를 한 번 수행한다. 기존 로컬 rclone 연결과 별개다. |
| Vast 관리 API 인증 | 사용자가 API 키를 발급하고 로컬의 Git 제외 경로에 안전하게 저장한다. 채팅·로그·ZIP에 넣지 않는다. 필요한 최소 권한을 확인한다. |
| CLI와 전송 실행기 | 요청받으면 agent가 로컬에 설치·구현한다. 이번 읽기 전용 확인에서 PATH의 Vast CLI와 기본 API 키 파일은 없었다. 다른 저장 위치까지 없다고 단정하지 않는다. |
| 연결·인스턴스 식별 | API 인증 후 agent가 연결 ID와 승인된 인스턴스를 조회한다. 매번 사용자가 연결 ID를 수동 입력하는 구조를 기본으로 삼지 않는다. |
| 최초 검증 | 소량 전송의 경로·권한·정지 상태 복사·재시도·완료 판정·SHA256을 검사한다. 속도·진행률 조회와 기존 파일 재사용 동작은 아직 미검증이다. |

인증과 검증이 끝나면 사용자 요청을 받아 원본 목록/목적지/디스크 확인 → 승인된
인스턴스 정지 → Cloud To Instance 복사 → 완료 확인 → 재시작 → 조각/결합 압축본/
해제 파일 검증을 묶는 실행기를 만들 수 있다. 학습 시작은 별도 승인이다. 자동으로
새 인스턴스를 임대하거나 destroy하지 않으며 원본을 삭제하는 sync 옵션은 사용하지 않는다.
현재 완성된 딸깍 실행기가 존재한다는 뜻은 아니다.
[공식 cloud copy CLI](https://docs.vast.ai/cli/reference/cloud-copy)는 연결 ID·인스턴스 ID·
원본/목적지·전송 방향 및 dry-run을 제공한다. 실제 설치 버전에서 옵션과 동작을 다시 확인한다.

**보안 계약 변경은 명시적으로 확인한다.** 현재 rclone은 갱신 토큰을 로컬에 보존하고
원격에 접근 토큰만 전달한다. Cloud Sync는 인증 정보를 호스트에 임시 복사하며 그
인증이 가진 권한으로 접근한다. 전용 client_id는 API 프로젝트를 분리하는 것이지
개인 계정의 파일 접근 범위를 제한하는 기능이 아니다. 전용 계정·권한 범위와 Secure
Cloud 호스트 여부를 검토한 후 사용자가 연결에 동의해야 한다. 기존 자격증명을
자동으로 Vast에 등록하지 않는다.

이번 합의는 문서화만 했다. CLI 설치·API 키 생성·Cloud Connections 등록·Cloud Sync
전송·인스턴스 정지/시작은 수행하지 않았다. 다음은 현재 rclone 다운로드 완료 후
무결성 검사와 해제이며, Cloud Sync는 이후 필요할 때 별도 요청으로 진행한다.

## 2026-10-10 Vast 템플릿 안내와 블로그용 증거 기록

이 절은 임대 전에 사용자가 공유한 화면과 안내한 설정을 보존하는 역사적 기록이다.
당시 사용자 텍스트에서 템플릿 이름과 Private 표시를 확인했으나, 정확한 이미지 tag와
실제 원격 실행은 확인 전이었다. 이후 임대·접속·설치는 위 원격 진행 기록에서 확인했다.
아래 안내값을 모두 실제 적용값으로 해석하지 않는다.
사용자는 향후 블로그에 선택 이유·실험 결과·캡처를 함께 사용할 예정이다.

### 화면에서 관측한 내용과 선택 이유

- 템플릿 목록의 `PyTorch (Vast)` 카드에는 CUDA 12.4가 표시됐다. 이 표시는 실제
  설치된 PyTorch 버전을 검증한 결과가 아니다. 로컬의 직접 설치 경로에서 사용하는
  PyTorch 2.7.1 / CUDA 12.8과 맞추기 위해 `+ New`에서 이미지 tag를 고정하도록 안내했다.
- 생성 화면에는 Image Path:Tag/Version Tag, SSH/Jupyter/ENTRYPOINT 모드,
  direct SSH, 시작 스크립트, 인증 정보, 컨테이너 디스크, Public/Private 선택이 있었다.
  공유된 생성 화면은 입력 전 상태였으며 SSH 모드와 Private가 선택되고 디스크는 8GB였다.
- `devel` 이미지로 nvcc 등 사용자 정의 CUDA 연산의 빌드 도구를 확보한다. 이 이미지는
  완성된 팀 BEVFusion 이미지가 아니므로 접속 후 `setup_direct_environment.sh`로 설치·검증한다.
- 템플릿은 컨테이너 이미지와 시작 설정을 지정하는 것이다. ZIP이나 Docker tar를
  업로드하는 절차가 아니다. 데이터와 코드는 나중에 별도로 전달한다.
  로컬의 팀 이미지가 원격 registry에 게시됐다고 가정하지 않는다.

### 안내한 입력값 — 적용 확인 전

| UI 항목 | 안내값 |
|---|---|
| Template Name | `AjouNice-Base5-Torch271-CUDA128` |
| Description | `Base5 BEVFusion training and inference benchmark` |
| Image Path:Tag | `pytorch/pytorch:2.7.1-cuda12.8-cudnn9-devel` |
| Version Tag | 위의 고정 tag를 확인한다. Automatic/latest는 사용하지 않는다. |
| Launch Mode | `Interactive shell server, SSH` |
| Direct SSH | 체크한다. 실제 후보의 외부 접속 포트 지원 여부를 확인한다. |
| Docker Options / 추가 Ports / Environment Variables | 비워 둔다. SSH 모드의 기본 포트를 사용한다. |
| On-start Script / Extra Filters | 비워 둔다. 접속 후 설치·검증을 실행한다. |
| Docker Repository Authentication | 공개 base 이미지이므로 비워 둔다. |
| Container disk | 첫 5K는 200GB, 같은 인스턴스에서 30K까지 학습한다면 500GB를 검토한다. |
| Add recommended volume settings | 미선택 |
| Visibility | Private |

`Create & Use`는 템플릿을 저장·선택한 뒤 GPU 후보 화면으로 이동하는 단계다.
GPU 임대와는 구분한다. 공유 메모리는 Vast에서 자동 할당한다. `--shm-size`를
Docker Options에 넣어 적용됐다고 가정하지 않고, 접속 후 `/dev/shm`과 할당 RAM을 확인한다.
컨테이너 디스크는 생성 시 고정되므로 임대 전에 데이터·압축본·가중치의 최대 용량을 산정한다.
[공식 템플릿 설정](https://docs.vast.ai/guides/templates/template-settings),
[공식 실행환경·공유메모리·디스크](https://docs.vast.ai/guides/instances/docker-environment).

### 학습 GPU와 배치 선택 논의 — 실측 전

추론 비용은 대회용 4090에서 측정하되, 학습은 다른 GPU에서도 수행할 수 있다.
시간당 가격뿐 아니라 데이터 로더·순전파·역전파·optimizer를 포함한 초당 이미지 처리량과
전체 비용을 비교한다. 메모리가 늘어난다고 연산 속도가 2배로 증가하거나 정확도가 좋아지는 것은 아니다.
배치를 키우면 같은 epoch에서 가중치를 업데이트하는 횟수가 줄어들므로 LR·warmup·학습 길이를 함께 검토한다.
배치 2 × 누적 2와 배치 4 × 누적 1은 명목상 유효 배치가 4인 비교 후보다. 다만
BatchNorm 통계 때문에 완전히 동일한 학습은 아니다. 실제 VRAM·처리량·수렴은 아직 확인하지 않았다.
NVIDIA 공식 4090은 24GB다. 48GB 후보는 단일 개조 카드인지, 2×24GB의 합산 표시인지 확인한다.
[공식4090 사양](https://www.nvidia.com/en-au/geforce/graphics-cards/40-series/rtx-4090/),
[gradient accumulation](https://docs.pytorch.org/docs/stable/notes/amp_examples.html),
[BatchNorm](https://docs.pytorch.org/docs/stable/generated/torch.nn.modules.batchnorm.BatchNorm2d.html).

### 캡처 원본 보관

Documents에서 템플릿 목록과 GPU 후보의 원본 **2장**을 확인하고 수정·재인코딩 없이
복사했다. 원본 파일은 그대로 보존한다. 채팅에 올린 생성 폼 4장의 별도 파일은 아직
확인하지 못했으므로 모든 캡처를 파일로 백업했다고 표현하지 않는다.

- 원본: `/home/luv/Documents/Screenshot from 2026-10-10 16-07-40.png`.
- 보관본: `artifacts/handoff/base5_vast_blog_evidence_261010/screenshots/01_vast_template_catalog_20261010_160740.png`.
- [캡처 목록 원본 보관본](../../artifacts/handoff/base5_vast_blog_evidence_261010/screenshots/01_vast_template_catalog_20261010_160740.png).
- GPU 후보 원본: `/home/luv/Documents/Screenshot from 2026-10-10 16-18-56.png`.
- [GPU 후보 원본 보관본](../../artifacts/handoff/base5_vast_blog_evidence_261010/screenshots/02_vast_4090_candidates_20261010_161856.png).
- 이미지와 관리용 manifest는 Git에서 제외한 산출물이다. 위 상대 링크는 로컬에서는
  열리지만 Git clone에 이미지가 포함된다는 뜻은 아니다. 별도의 외부 업로드는 하지 않았다.
- 다음에는 실제 저장된 템플릿의 상세 설정, 임대한 GPU의 사양과 비용,
  접속 후 환경 검증, 추론·학습 측정 결과를 각각 캡처와 로그로 추가한다.
  공개 전에는 토큰·결제 정보·접속 정보를 다시 검사한다.

### GPU 후보 추천 — 2026-10-10 16:18:56 KST 캡처 기준

사용자가 제공한 텍스트와 위 GPU 후보 캡처를 함께 확인했다. 텍스트와 캡처의
offer 번호는 다르지만 머신·호스트 번호가 같아 같은 추천 대상으로 식별했다.
이 표는 후보 추천 기록이며 예약이나 임대 완료 기록은 아니다.

| 순위 | 머신 / 호스트 | 텍스트 offer → 캡처 offer | GPU·할당 자원 | 표시 가격 / 24시간 | Reliability |
|---|---|---|---|---|---|
| 1 | m:37552 / host:135676, 일본 | #48470915 → #48470909 | 1× RTX 4090, GPU당 48GB / CPU 16 / RAM 약 97GB | $0.462/h / $11.09 | 99.73% |
| 2 | m:38442 / host:124072, 일본 | #45473509 → #45473511 | 1× RTX 4090, GPU당 48GB / CPU 16 / RAM 약 97GB | $0.463/h / $11.11 | 99.36% |

가격은 사용자가 200GB 디스크 포함으로 조회했다고 설명한 표시값이며, 대역폭 요금은 별도다.
실제 임대 직전에는 이미지 tag, 컨테이너 디스크 200GB, 전송 단가와 총시급을 다시 확인한다.
카드의 470.9GB / 652.5GB는 컨테이너에 지정한 용량이라고 해석하지 않는다.
두 후보 모두 Max CUDA 12.8로 표시되지만 실제 CUDA 연산의 설치·실행 여부는 접속 후 검증한다.

사용자의 이번 목표는 하루 안에 5K 추론 실험과 임시 모델 학습을 수행하는 것이다.
48GB 후보는 가격 차이가 작고 실제 배치 확대를 시험할 여유가 있어 추천했다.
다만 학습 완료 시간을 보장하지 않으며, 짧은 학습 처리량을 측정한 뒤 epoch를 정한다.
추론 실험은 대회용 24GB에서 실행 가능한 메모리 범위를 별도로 확인하고, 이 후보의
추론 시간을 대회 PC의 확정값으로 사용하지 않는다. DLPerf는 우리 모델의 실측 처리량이 아니다.

사용자 텍스트 상단의 `Jupyter` 표시는 처음 안내한 SSH-only와 다르다. Jupyter+SSH도
SSH 접속을 제공하므로 작업을 막는 문제는 아니다. 정확한 실행 모드와 direct SSH 설정은
실제 저장된 템플릿 또는 접속 화면에서 확인한다. 계정의 비밀 키나 결제 정보를 캡처하지 않는다.

## 2026-10-10 Sunny13 5K Drive 원본 백업

사용자가 데이터셋 백업 업로드를 별도로 요청했다. **2026-10-10 15:41:53 KST에 Drive 백업과
원격 검증이 완료됐다.** 아래 'Drive 업로드 미실행'은 이 요청 전 로컬 준비의 시점 기록이다.
데이터셋 백업 완료와 학습 체크포인트의 외부 백업 검증은 구분한다.

- 대상은 최종 v140에서 선택한 5000장의 원본 카메라·LiDAR·GT, 시나리오·네트워크·
  측위/상태 기록·출처 정보 및 448/512 입력용 파생 infos다.
  실패하거나 선택하지 않은 캡처, 가중치, 코드, 자격증명은 포함하지 않는다.
- 원본 파일 합계는 26,591,499,150 bytes(24.765GiB), 173,290파일이다. train 4255 / val 745다.
- zstd level 19 / T8로 압축한 크기는 **19,438,853,713 bytes(18.104GiB)**다.
  PNG 내용은 변경하지 않고 tar 바이트를 재압축했다. 기존 level 3보다
  179.38MiB(약 0.96%)만 추가로 줄었다. PNG 자체가 이미 압축돼 있어 강한 압축의
  추가 이득은 작다. 이후 전송에는 이 압축본을 그대로 재사용한다.
- 압축 CRC 전수 검사와 다시 해제한 tar 스트림의 SHA256 동일성 확인을 완료했다.
  archive SHA256: `ab24307ee17007c4170dacabb00e6c97130e090ddbdb45f0e99c38b244a8374f`.
- 로컬: `artifacts/backups/base5_sunny13_5k_v140_261010/`.
  Drive: `AjouNice2026/dataset/Base5/sunny13_5k_v140_261010/`.
  대용량 단일 파일 전송은 기존 v1.50.2와 별도로 설치한 v1.75.2 모두에서
  최종 저장 단계의 전체 재전송 현상을 보였다. 정확한 원인은 아직 단정하지 않는다.
  원본 압축파일을 1GiB 이하의 19조각으로 나눠 전송했다. 조각을 결합한 SHA256은
  위 압축파일과 동일하다. 로컬 분할본은 위 폴더의 `parts/`에 있다.
  rclone v1.75.2 / 8MiB chunk / 2 transfers로 각 조각의 저장 완료를 확인했다.
  시스템 rclone은 교체하지 않았다. **19/19 조각의 원격 크기·MD5·SHA256 메타데이터가
  일치했다.** 메타데이터를 포함한 24파일의 rclone check에서 차이 0건,
  `COMPLETE.json`의 추가 전송·검증에서도 1파일의 차이 0건을 확인했다.
  [실제 Drive 백업 폴더](https://drive.google.com/drive/folders/17RebmPKe_q8id49dm5waGs1iPzov_YnH).
  원격 파일 전체를 다시 다운로드한 검증은 아니다. Drive가 제공하는 SHA256과
  로컬 값을 대조했다. 다운로드 후에는 `RESTORE.txt`에 따라
  `sha256sum -c SHA256SUMS`, 조각 결합 SHA256 검증, 스트림 해제를 수행한다.
- 분할 전송은 재시도를 포함해 827.14초(13.79분), 평균 22.41MiB/s였다.
  앞선 단일 파일의 실패 시도와 압축·검증 시간은 포함하지 않는다.
  압축, CRC 검사, 로컬 SHA·MD5 확정은 14:29:45~14:45:39에 수행했다.
  분할 로그에서 **공용 API 프로젝트의 요청량 한도 오류(403)**를 확인했다.
  계정의 저장 용량 한도 오류가 아니다. 앞선 단일 파일 실패도 같은 원인이었는지는
  당시 상세 로그가 없어 단정하지 않는다. `parts_upload_debug.log`와
  `parts/COMPLETE.json`을 근거로 보존한다. 장기 체크포인트 백업 전에는
  사용자 본인의 Google API client_id를 설정하고 재검증한다.
  새 Cloud 프로젝트나 OAuth 권한을 임의로 생성하거나 공유 권한을 변경하지 않았다.
- 원격에서는 압축본과 해제본을 동시에 보관할 약 43GiB와 추가 학습 공간을 확보한다.
  백업 전 Drive 계정의 잔여 용량은 약 94GiB였다. 6개 환경이 비슷한 크기라면
  합계 약 109GiB가 필요하므로 나머지 25K를 백업할 때 잔여 용량을 다시 확인한다.

### Vast 전송 방식 선택

`rsync`는 압축 여부를 정하는 도구가 아니다. 원본 디렉터리와 tar.zst 파일 모두 전송할 수 있다.
로컬에는 단일 tar.zst도 그대로 남아 있으므로 SSH로 이 파일을 보낼 수 있다.
Drive에는 19조각과 복원 설명, 조각별 체크섬과 결합 체크섬을 보관한다.
173,290개 파일을 개별 전송하는 대신 압축본을 `rsync -aP`로 전달한다.
이미 압축된 파일이므로 추가 압축 옵션 `-z`는 기본적으로 사용하지 않는다.
전송 재개와 다운로드 후 SHA256 검증을 함께 사용한다.
[rsync 공식 설명](https://download.samba.org/pub/rsync/rsync.1).

Drive 백업을 완료한 뒤에는 Drive→Vast 다운로드로 집의 업로드 회선을 피할 수 있다.
gdown을 사용하려면 공유·다운로드 권한을 확인해야 하며, 백업을 임의로 공개하지 않는다.
비공개 팀 폴더는 인증된 rclone으로 다운로드할 수 있다. 다만 이번에 관측한
공용 client_id의 요청량 한도 오류를 고려해 첫 Vast 전송은 **로컬 단일 압축파일의
직접 rsync 전송**을 우선 권장한다. Vast에 연결한 뒤 기존 분할파일로 짧게 전송하여
Drive 다운로드와 비교한다. Drive 쪽이 충분히 빠르고 인증·한도 문제가 없으면
다운로드 경로로 바꾼다. 업로드에서 관측한 한도 오류가 다운로드에도 발생한다고
단정하지 않는다. 현재 Vast 전송 속도는 측정하지 않았다.
[gdown 공식 설명](https://github.com/wkentaro/gdown), [rclone Drive 인증·hash 설명](https://rclone.org/drive/).

| 지속 실효속도 가정 | 18.104GiB 네트워크 전송만 |
|---|---:|
| 100Mbps | 약 25.9분 |
| 300Mbps | 약 8.6분 |
| 1Gbps | 약 2.6분 |

압축 해제, 해시 검사, 초기 연결 비용은 별도다. 처음부터 Drive를 경유하면
업로드와 다운로드의 두 단계가 필요하지만, 백업을 이미 완료한 현재는
Vast로 전달하는 한 단계만 비교한다.

## 2026-10-10 로컬 준비 결과

이 절은 Drive 백업 요청 전의 로컬 준비 기록이며, 위 15:41의 데이터셋 백업 완료 기록보다
이전 상태다. 아래 10-09의 'direct runner/백업 미구현' 표기보다 우선한다.
MORAI와 ROS를 종료한 상태에서 준비했다. **당시에는 원격 임대·전송·Drive 업로드·
4090 측정·본학습을 수행하지 않았다.**

| 항목 | 구현·검증 상태 | 남은 확인 |
|---|---|---|
| 성공 5K 독립 전달본 | 5,000장 / train 4,255·val 745. 448·512 입력에서 동일 split을 사용하며 원본 센서·annotation·메타데이터를 보존한다. | 원격 해제 후 전수 SHA256 검사 |
| 실제 용량 | 원본 26,591,499,150 bytes = 24.765GiB / 173,290파일. 당시 level 3 압축본은 19,626,947,031 bytes = 18.28GiB였다. | 당시 압축본과 해제본의 동시 보관에 약 43.1GiB 필요 |
| 직접 실행 학습기 | Docker를 중첩하지 않는다. 데이터·설정·코드·환경 해시를 고정하고 warmup과 cosine 구간을 분리한다. | 실제 4090에서 배치·epoch·LR·최종 crop 선정 |
| 저장·재개 | .partial→.pth의 원자적 저장, optimizer·scheduler·AMP·메인 RNG 보존, 변경된 실행 계약 거절 | 원격의 동일 환경에서 재개 확인. epoch 중간이나 worker 순서의 완전한 재현은 보장하지 않음 |
| 실제 모델 smoke | 실제 2장·4클래스·448 입력·AMP로 4 iteration 실행. epoch 1 저장 후 중단하고 epoch 2로 재개하는 시험에 통과했다. | 수렴·정확도 시험은 아님 |
| 백업기 | 완료 파일의 snapshot→SHA256 검증→COMPLETE/LATEST 처리, 실패 재시도·중복 생략. 실제 smoke 체크포인트 2개의 로컬 복사에 통과했다. | 체크포인트용 rclone 인증·원격 권한·실제 Drive 재다운로드 검증은 미실행 |
| 초기화 대안 | nuScenes 카메라 backbone/neck의 29,142,666개 값을 동일하게 로딩했다. 용량은 111.4MiB다. | LiDAR/BEV 사전학습 가중치의 축 변환은 미검증. 최종 기본 초기화로 자동 선택하지 않음 |
| 실행 환경 | 기존 팀 이미지에서 CUDA NMS와 BEVFusion import 검사에 통과했다. 동일 base용 직접 설치 스크립트를 준비했다. | 새 Vast 환경의 전체 설치 시험과 이미지 게시·pull은 미완료 |

전달 폴더:

- `/home/luv/AjouNice2026/artifacts/handoff/base5_sunny13_portable_261010`: 원격 학습용 상대경로본이다.
  로컬에서는 하드링크로 원본과 inode를 공유한다. **이 폴더에서 파일을 수정하지 않는다.**
  원격에 복사하거나 압축을 해제하면 독립 파일이 되며 로컬 원본 경로에 의존하지 않는다.
- `/home/luv/AjouNice2026/artifacts/handoff/base5_remote_code_261010/base5_sunny13_portable.tar.zst`: 위 폴더 전체의 무손실 압축본이다.
- 같은 폴더의 `base5_remote_code.tar.gz`: detector/plugin/common, MMDetection3D fork와 설치 스크립트만 포함한다.
  ROS, planning, 전체 rosbag, MORAI는 원격 학습의 필수 입력에 포함하지 않는다.
- `artifacts/detection3d/bevfusion/pretrained/base5_nuscenes_camera_only_261010.pth`:
  카메라 사전학습 가중치만 사용하는 초기화 대안이다. 출처는 nuScenes의 기존
  MORAI3 adapter이며 E25에서 학습한 가중치가 아니다. 분류 head를 재사용하지 않으므로
  Animal 클래스는 새로 초기화한다.

원래 최종 v140 폴더의 343MiB는 원본 센서 자체가 아니라 선택 메타데이터다.
이 폴더만 전송해서는 학습할 수 없다. 독립 전달본은 선택된 5000개 캡처의 모든 파일을 포함한다.
출처 정보의 과거 절대경로는 감사용이며 학습 입력은 `data/<token>/...`다.
합성 데이터의 test infos는 **빈 목록**이다. rosbag test를 생성했다는 뜻은 아니다.

### 임대한 환경에서의 순서

SSH 정보와 선택한 이미지를 확인한 뒤 실행한다. 아래 경로는 원격에서 압축을 해제한
저장소 root를 뜻한다. Vast 자체가 컨테이너이므로 명령 앞에 `docker run`을 붙이지 않는다.

```bash
cd "$HOME/AjouNice2026"

# 팀이미지가 이미 있으면 검사만. 일반base에서는 아래 --install 경로를 사용.
bash docker/bevfusion/scripts/setup_direct_environment.sh --check-only

# 정확히 pytorch/pytorch:2.7.1-cuda12.8-cudnn9-devel 계열의 빈환경인 경우만:
# bash docker/bevfusion/scripts/setup_direct_environment.sh --install

export PYTHONPATH="$PWD:$(python -c 'from pathlib import Path; import mmdet3d; print(Path(mmdet3d.__file__).resolve().parents[1])')"
export TORCH_CUDA_ARCH_LIST=8.9

python perception/detection3d/bevfusion/tools/prepare_base5_portable_dataset.py \
  --verify "$HOME/base5_sunny13_portable_261010"
```

`--install`은 PyTorch 2.7.1 / CUDA 12.8과 CUDA devel/nvcc를 확인하고,
템플릿이 다르면 중단한다. MMCV 2.1.0, 팀 fork, 사용자 정의 CUDA 연산을 설치한다.
로컬 팀 이미지의 tag는 registry에 게시되지 않았으므로 현재 pull 가능한 이미지라고
안내하지 않는다. 새 환경에서 설치가 실패하면 원인을 확인하고 진행하며,
인스턴스를 임의로 destroy하지 않는다.

4090에서 최종 입력과 배치를 확정한 뒤 실행 manifest를 생성한다. 다음 명령은
**준비만 수행하는 예시**다. 12 epoch / 배치 2 × 누적 2는 확정된 본학습 설정이 아니다.
448·512 입력은 그에 맞는 config와 profile을 함께 선택한다.

```bash
python perception/detection3d/bevfusion/tools/run_base5_direct_training.py \
  --config perception/detection3d/bevfusion/configs/bevfusion_base5_dense_448_separated_candidate.py \
  --data "$HOME/base5_sunny13_portable_261010" --profile 448x896 \
  --work-dir "$HOME/ajou_runs/base5_sunny13_candidate" \
  --epochs 12 --batch 2 --accumulate 2 --workers 4 \
  --init-checkpoint "$HOME/base5_nuscenes_camera_only_261010.pth"
```

설정을 확정한 뒤 같은 인자에 `--run`을 추가한다. 준비만 실행했고 계약이 같으면
같은 work-dir를 사용할 수 있다. 실제 학습을 시작했거나 설정이 바뀐 경로를 새 학습으로
덮어쓰지 않는다. 준비 후 설정을 변경했다면 새 work-dir를 사용한다.
실제 학습을 재개할 때는 같은 인자에 `--resume --run`을 붙인다.
초기 체크포인트 인자는 생략할 수 있지만 데이터·crop·배치/누적·epoch/LR/seed·코드·패키지는
같아야 한다. 5K→30K 확장을 동일 학습의 resume으로 연결하지 않는다.

epoch의 iteration 수는 `accumulate`로 나누어떨어져야 한다. 학습 데이터 4255장에서
배치 1 / 누적 4는 거절하고, 배치 2 / 누적 2는 2128 iteration / 1064 update로 통과한다.
epoch 경계에 저장하지 않은 gradient가 남아 재개 시 유실되는 상황을 피하기 위한 제약이다.
마지막 배치의 실제 샘플 수는 명목 유효 배치보다 작을 수 있다.

외부 백업은 별도의 tmux pane에서 실행한다. rclone을 설치하고 계정을 설정해야 한다.
인증 파일은 Git이나 전달용 ZIP에 넣지 않는다.

```bash
python perception/detection3d/bevfusion/tools/backup_base5_checkpoints.py \
  --work-dir "$HOME/ajou_runs/base5_sunny13_candidate" \
  --destination ajou_drive:model/bevfusion/base5/checkpoint_backups \
  --drive-root-folder-id 12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP \
  --interval-sec 120
```

원격 복사 뒤 `rclone cat`으로 파일을 읽어 SHA256을 검증하므로 체크포인트마다
추가 다운로드가 발생한다. 해시 주소·COMPLETE·설정 파일의 검증을 마친 뒤 LATEST를 발행한다.
원격 보관본을 자동 삭제하지 않으므로 Drive 여유 공간과 전송 비용을 확인한다.
오류는 `backup_status.json`에 기록하고 다음 주기에 재시도한다.
자격증명은 stderr나 로그에 출력하지 않는다. epoch가 길면 허용 손실 시간을 다시 정한다.
현재 구현은 epoch 경계에서 저장한다.

### 로컬 정리 결과와 보존 원칙

- `~/Music/new_rosbag`의 중복 ZIP 6개는 정상 추출본의 66파일 전체와 해제 후 SHA256을
  대조한 뒤 삭제했다. **9,432,764,756 bytes = 8.785GiB**를 회수했다.
  유효 bag과 검증·메타데이터는 `rosbags/eval/perception/260923_teammate_manual`에 보존했다.
- `~/Music/rosbag0923`의 잘못 받은 메타데이터 전용 50파일도 정상본과 모두 동일해 삭제했다.
- helper 누락으로 실패한 preview와 첫 최종 benchmark의 중복 ZIP·내용물은 삭제했다.
  작은 manifest와 실패 로그는 `artifacts/cleanup/failed_input_pool_bundles`로 옮겼다.
  **검증된 final_v2 묶음은 보존한다.**
- 초기 직접 실행 smoke의 4-step 학습 가중치와 그 로컬 백업 약 1.85GiB도 검증 후 제거했다.
  메타데이터와 검증 로그는 `artifacts/cleanup/base5_smoke_v1_metadata_only.tar.gz`에 보존했다.
  최신 v2 smoke는 유지한다. 본학습 체크포인트나 원본 캡처를 삭제한 것이 아니다.
- 과거 캡처 폴더가 v140 선택의 원본인 경우에는 삭제하지 않았다. Base3/Base4 원본,
  nuScenes/E25 체크포인트, 유효 bag, 사용자가 보존을 요청한 yaw 백업과 재현 근거를 유지했다.
- 오래된 진행 설명은 문서 안내에서 과거 이력으로 분류했다. temporal 소스는
  import와 재현에 필요한 의존성이 있어 삭제하지 않고 기본 OFF 상태를 유지한다.
  데이터셋·산출물·연구보고서의 Git 제외 정책도 유지한다.

검증 근거는 `artifacts/detection3d/bevfusion/base5_direct_resume_smoke_261010_v2/report.json`,
카메라 초기화 보고서, 위 메타데이터 전용 압축본의 로컬 백업 상태,
`artifacts/cleanup/261010_verified_duplicate_rosbag_zips.json`이다.
과거 Torch 2.7 + MMEngine에서 `weights_only` 기본값 때문에 전체 학습 상태 재개가 실패한
문제는 전용 `Base5AtomicRunner`의 **신뢰된 로컬 체크포인트** 로딩에 한정해 수정했다.
전역 torch 기본값은 변경하지 않는다.

## 임대 사양 추천

아래는 초기 사양 안내다. 2026-10-10의 실제 후보 비교에서는 가격 차이가 작은 단일 48GB
4090을 우선 추천했다. 구체적인 후보는 위 'GPU 후보 추천' 절을 따른다.

| 항목 | 첫 5K 추천 | 이유와 확인 사항 |
|---|---|---|
| GPU | RTX 4090 24GB 한 장 | 대회 GPU와 같은 계열에서 학습·추론 비용 확인; batch와 최대 메모리는 별도 smoke |
| 임대 유형 | On-demand, verified host 우선 | 첫 환경 구축·재개 검증 중 회수 위험을 줄임; 장애·잔액 부족·계약 만료까지 없어지는 것은 아님 |
| CPU | 할당 8 vCPU 이상, 가능하면 16 이상 | 이미지 decode/augmentation과 loader 병목 확인; 전체 호스트 core 수와 할당분 구분 |
| RAM | 32GB 최소 후보, 64GB 이상 권장 | loader worker/prefetch 및 가공·평가 여유; shared memory도 검사 |
| 디스크 | 첫 5K는 200GB, 같은 임대로 30K까지 계속하면 500GB부터 검토 | 완성 학습본·이미지/환경·checkpoint·압축 중복·평가 subset까지 계산 후 확정 |
| 디스크·네트워크 | 빠른 로컬 SSD, direct SSH, 낮은 전송 단가 | GPU 가격뿐 아니라 업로드 시간과 loader I/O도 비교 |
| 안정성 | reliability 0.99 이상 선호, 계약 잔여기간 넉넉히 | 표시 점수는 장애 보장이 아님; 실제 1차 smoke/네트워크/지속 실행 확인 |
| CUDA 환경 | 팀 PyTorch2.7.1/CUDA 12.8 계열과 호환 | host driver와 CUDA extension sm89 지원, 설치 버전/digest 기록 |

On-demand와 Interruptible은 다른 임대 유형이며 Interruptible은 회수로 정지될 수 있다.
첫 실행은 On-demand로 하고, resumable checkpoint와 외부 백업이 검증된 뒤 중단형의
가격 이득을 검토한다. 계약 만료일·잔액·host 안정성은 On-demand에서도 확인한다.
[공식 임대 유형](https://docs.vast.ai/guides/instances/choosing/instance-types).

## 2026년10월9일 가격 관측

18:20 KST 무렵 공식 search offers API를 읽기 전용 조회했다.
필터는 verified/rentable/not-rented, 단일 4090, RAM 32GB 이상, 할당 8 vCPU 이상,
disk 200GB 이상, reliability 0.98 이상, On-demand, allocated_storage200GB이며
dph_total 오름차순 12개다. 따라서 전체 시장의 평균·최저가가 아닌 해당 조건의 후보 관측이다.
[공식 검색 API](https://docs.vast.ai/api-reference/search/search-offers),
[실시간 선택 화면](https://cloud.vast.ai/create/).

| 관측 후보 | 지역 | 할당 vCPU / RAM 표시 | GPU 기본 $/h | 200GB 포함 dph_total $/h |
|---|---|---|---:|---:|
| 39308422 | 일본 | 24 / 약 129GB | 0.3867 | 0.4126 |
| 46384000 | 한국 | 16 / 약 64GB | 0.4133 | 0.4504 |
| 45473516 | 일본 | 16 / 약 97GB | 0.4133 | 0.4626 |

조회 12개 total은 약 $0.413~0.552/h였다. 저가 후보의 storage_cost는 각각
$0.0933/$0.1333/$0.1773 per GB/month로 표시됐다. dph_total에는 이번 200GB 할당 비용이
이미 반영되어 있으므로 디스크를 다시 더하지 않는다. 전송은 별도이며 한국 후보의
inet_down_cost/inet_up_cost 표시값은 각각 약 $0.0002604/$0.0003906 per GB였다.
최종 checkout에서 할당량·전송 단위와 총액을 다시 확인한다. 한국 후보는 가용 disk가
206GB로 표시되어 500GB 장기 구성에는 해당 시점 사용 불가였다. 후보 ID는 예약이나 추천 확정이 아니다.

같은 기본 조건의 3090 저가 3개는 200GB 포함 약 $0.202~0.256/h였다. 가격 대안이지만
우리 확대 모델의 4090 대비 학습 속도·시간당 효율은 아직 측정하지 않았다. 대회 추론 비용까지
동일 GPU에서 확인하려는 첫 실행은 4090을 우선한다.

예산은 여유를 두고 **4090+200GB $0.45~0.65/h**로 잡는다. 이는 추천 예산이지 확정 요금이 아니다.

| 가동 시간 가정 | GPU와 200GB 예산 | 별도 항목 |
|---|---:|---|
| 24시간 | $10.8~15.6 | 데이터 전송·유휴 저장·세금/결제 비용 |
| 48시간 | $21.6~31.2 | 동일 |
| 72시간 | $32.4~46.8 | 동일 |

설치·업로드·실험 중 실행 상태의 시간도 예산에 포함한다. 실제 학습시간은 아래 짧은
step 실측으로 갱신한다. 전체 비용은 실행시간×해당 총시급 + 정지 중 저장 + 전송 등이며,
가격·재고는 변한다. [공식 비용 구성](https://docs.vast.ai/guides/instances/pricing).

## 저장공간과 업로드 범위

당일 대시보드 기존 저장량 표본은 승인 3208장에 21.84GiB, 5K 전체 추정 34.05GiB였다.
이는 실패·재시도까지 포함한 캠페인 저장량 외삽이며 최종 학습 전달본의 실측이 아니다.
30K 단순 6배는 약 204GiB지만 환경별 이미지 크기·불필요 artifact·후처리에 따라 달라진다.
5K 종료 후 사용할 파일 목록과 byte 합계를 확정하고 GB/GiB 차이를 고려해 임대한다.

- 원격에는 선택한 성공 5K의 학습 입력·native annotation/필요 metadata·고정 split/info,
  호환 초기 weight, 코드 revision 및 소량의 실제 평가 raw를 우선 전송한다.
- 전체 MORAI SensorData·실패/연구 캡처·모든 rosbag·전달용 Docker tar를 무조건 복사하지 않는다.
  GT 재가공에 필요한 원본과 metadata는 로컬에 보존하며 원격에서도 학습 정합 감사 자료를 유지한다.
- rsync 재개 또는 분할 archive+SHA256을 사용한다. JPEG/PNG를 불필요하게 decode해
  부풀리거나 lossily 재압축하지 않는다. archive와 해제본이 동시에 존재하는 최대 용량을 계산한다.
- 200GB는 첫 5K 여유안이다. 이어서 30K까지 한 인스턴스에서 쓰려면 500GB 안팎을 검토하되
  archive를 원격에 모두 남기는 구성이라면 더 필요할 수 있다.

Vast container disk는 생성 시 크기가 고정되고 stop 후에도 요금이 발생한다. destroy하면
컨테이너 데이터는 삭제된다. 별도 volume도 특정 물리 host에 묶이므로 외부 백업을 대체하지 않는다.
[공식 저장공간 설명](https://docs.vast.ai/guides/instances/storage/types).

## SSH와 실행 환경

이 절은 초기 2026-10-09 계획을 보존한다. 직접 실행 학습기와 백업기는 이후 로컬에서
준비·검증했으며 실제 원격 설치·재개 검증은 남아 있다. 현재 명령은 위 '임대한 환경에서의
순서'를 따른다. 아래의 '아직 준비하지 않았다'를 최신 상태로 재사용하지 않는다.

Vast는 Docker image와 SSH launch mode를 지정할 수 있다. 기본 인스턴스 자체가
컨테이너이므로 우리 host용 runner의 docker run을 그대로 중첩 실행하지 않는다.
팀 이미지를 registry에서 받을 수 있게 준비하고 SSH 환경에서 직접 train.py를 실행하는
remote runner를 만드는 쪽을 우선한다. 현재 로컬 이미지 tag만 있다고 원격에서 pull 가능한 것은 아니다.
이미지 게시·접근 범위는 별도 결정하고 실제 digest와 CUDA ops를 확인한다.
[공식 template 설정](https://docs.vast.ai/guides/instances/choosing/templates).

기존 [team runner](../../perception/detection3d/bevfusion/tools/run_team_experiment.py)는
source snapshot·checkpoint hash·resume 및 W&B 설정을 지원하지만 docker run 기반이다.
원격 direct runner, 주기적 Drive uploader와 원격 재개 smoke는 아직 준비하지 않았다.
원격에서도 동일 resolved config·run manifest·submodule revision·최종 전처리를 기록한다.

사용자는 계정에 공개 SSH key를 등록하고 임대 화면의 실제 host/port를 전달한다.
VS Code Remote-SSH는 선택 가능하며, 로컬 터미널의 SSH로 원격 작업을 수행하는 방식도 가능하다.
연결 후 원격 repo·데이터가 자동으로 생긴다고 가정하지 않는다. private key는 로컬에 두고,
W&B/Drive/API 자격증명을 Git·전달 ZIP·실행 로그에 넣지 않는다.
[공식 SSH와 VS Code 연결](https://docs.vast.ai/guides/instances/connect/ssh).

작업은 원격 tmux에서 실행하고 VS Code/SSH 연결 종료와 학습 종료를 분리한다.
편집할 때는 원격과 로컬 어느 checkout인지 확인하고 실행 중 code/config는 고정한다.
설치된 ROS 풀스택은 첫 학습의 필수조건이 아니다.

## 로컬에서 원격 학습까지

| 단계 | 실행 위치 | 수행과 완료 기준 |
|---|---|---|
| 1 | 로컬, 이번 수집/MORAI 종료 후 | 무결성·split·crop·투영·cache/pooling 출력/gradient 정합; 비용 후보 축소 |
| 2 | 로컬 | 전달 5K·GT/info·코드·초기화·시험 raw manifest 확정, 원격 image/runner 준비 |
| 3 | 사용자 임대 후 원격 4090 | SSH·CUDA/spconv/custom ops·loader·W&B·disk/공유메모리 smoke |
| 4 | 원격 4090, 학습 전 | batch 1 후보 추론 latency와 VRAM, 동일 공간/depth·class/query 조건; 입력과 fallback 선택 |
| 5 | 원격 4090 | 실제 GT로 batch 1/2 및 가능한 4의 짧은 forward/backward/optimizer 시험; AMP skip·OOM·loader 비용 검사 |
| 6 | 원격 4090 | 호환 nuScenes 초기화 로딩 감사, checkpoint 저장→외부 백업→중단/재개 확인 후 Sunny13 5K 학습 |
| 7 | 원격 추론 + 로컬/원격 CPU 평가 | checkpoint 후보의 mission dev-val detection/tracking/Map-CV 평가; 실패 거리·종류로 원인 정리 |
| 8 | 이후 6 환경 30K | Sunny13 보존·나머지 25K 새 geometry/split 감사; 공간 group 일관성과 초기화/finetune/resume 차이를 명시 |

4090 추론은 MORAI 없이 실제 raw의 근접·원거리·밀집 subset에서 cold/warm,
모델 단독/CPU 준비 포함 p50/p95/p99·peak VRAM을 기록한다. GPU 모델이 같아도
host CPU·전력·PCIe·디스크가 달라 최종 대회 풀스택 실시간성 인증과는 구분한다.
평균 70ms가 아니라 13Hz≈76.9ms의 전체 경로 여유와 tail latency를 고려한다.
팀원 4090 결과와도 [동일 조건](261009_RTX4090_CAPACITY_RESULTS.md)을 맞춰 대조한다.

기본은 4-camera+LiDAR single-frame·7D·4-class, OD velocity/temporal OFF다.
물리 batch는 실측 후 결정하고 필요한 경우 accumulation을 쓴다. LR·warmup·scheduler는
최종 epoch/optimizer update 수에 맞춰 다시 검증한다. 짧은 smoke 스케줄을 본학습에
그대로 쓰거나, 추론의 GPU 속도 배수를 학습 ETA로 사용하지 않는다.
초기 5K의 epoch와 본 30K epoch는 아직 미확정이다. 짧은 측정에서
epoch 시간≈ceil(train 수/물리 batch)×iteration 시간+val/저장 시간으로 계산한다.

## Rosbag dev-val과 최종 평가

합성 5K의 train 4255/val 745를 유지하고 기존 반복 mission bag은 별도 dev-val로 사용한다.
첫 Sunny13 학습은 6 환경 최종 성능을 대표하지 않는다. raw가 없는 bag은 새 detector를
재추론할 수 없으므로 sensor 포함 여부부터 확인한다.

bag→평가용 센서/GT info 가공은 가능하지만 전처리·timestamp·asset catalog/이름 없는
제원 복구·GT 속도 계약을 그대로 맞춘다. 인접 프레임을 무작위 train/val로 나누거나
사용한 bag을 최종 독립 test로 다시 부르지 않는다. 평가 프레임·범위·threshold를 고정해
E25와 비교하고 threshold 튜닝은 dev-val에서 한다. 필요하면 원격은 detector 출력만
저장하고 기존 CPU tracker/predictor·평가기는 로컬에서 실행한다. 새 checkpoint에 대한
추론 결과이지 rosbag에 과거 저장된 detection을 재사용하는 평가가 아니다.

장기 학습마다 모든 bag을 돌릴 필요는 없다. offline val로 후보를 좁힌 뒤 선택한
checkpoint에 같은 mission detection/tracking/Map-CV를 실행하고, 최종은 미사용 주행으로 확인한다.
[split·평가 계약](../../dataset/morai_od/docs/261009_base5_training_split_visibility_and_temporal_decisions.md).

## 중단 복구와 종료 기준

- 최소 epoch마다 full checkpoint, 길어진 epoch는 15~30분 손실 예산에 맞춰 중간 저장도 검토한다.
  model·optimizer·scheduler·AMP scaler·epoch/global step·가능한 RNG 상태를 보존한다.
- checkpoint 완료 후만 비동기 Drive 업로드하고 SHA256·완료 marker를 남긴다.
  latest resume본과 best 후보·config/manifest/로그를 보관한다. 미완료 파일을 latest로 취급하지 않는다.
- 실제 중단→재개에서 LR·update·optimizer/scaler가 이어지는지 확인한다. mid-epoch batch와
  sampler 순서까지 완전히 재현되는지는 별도로 검증하며 기본적으로 보장하지 않는다.
- 같은 실행의 resume과 새학습/finetune의load_from을 구분한다. 5K→30K는 dataset/schedule이
  달라지므로 자동 resume하지 않고 다음 학습의 초기화와 스케줄을 별도 결정한다.
- 잔액·계약 만료·disk·backup 실패를 감시한다. instance destroy는 사용자 승인 후,
  로컬/Drive에서 weight·resume본·증거의 무결성을 확인한 뒤 한다. On-demand도 외부 백업은 필수다.

위 목록은 초기 계획 당시의 준비 항목이다. 2026-10-10에는 전달 용량, 직접 실행 학습기,
백업기와 로컬 재개 시험을 준비했다. 최종 입력과 초기화 선택, 실제 원격 환경에서의
설치·백업·재개 검증은 남아 있다. 본학습 설정·예상 시간·비용을 보고한 뒤 진행한다.
이 문서 작성만으로 임대나 업로드를 시작하지 않는다.
[상위 묶음 실험 계획](261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md).

## 원격 MORAI는 학습 환경과 별도 검토

2026-10-09 기본 용도는 MORAI 없는 오프라인 학습과 추론 비용 시험이다. 원격 MORAI는
별도 소규모 실행 시험으로 분리하고 이를 위해 본학습을 늦추지 않는다. 실행 가능성은
대회용 MR에서 아직 검증하지 않았다. GPU 연산 능력 외에도 컨테이너의 GPU 그래픽/Vulkan,
원격 화면, 계정 인증·동시 실행 허용 범위 및 ROS/UDP 포트 매핑을 확인해야 한다.
[MORAI 공식 설치 안내](https://help-morai-sim-en.scrollhelp.site/morai-sim-standard-en/installation-and-setup),
[Vast 컨테이너·네트워크 안내](https://github.com/vast-ai/docs/blob/main/guides/instances/docker-environment.mdx).

원격 자동 수집을 시험한다면 MORAI와 캡처기를 같은 원격 환경에 두고 파일만 전달하는
구성을 우선 검토한다. 카메라·LiDAR를 인터넷으로 로컬에 보내는 방식의 대역폭·지연을
현재 localhost/LAN 계약과 같다고 가정하지 않는다. 동일 GPU의 MORAI/학습 병행은
경합과 latency 측정 왜곡 가능성이 있어 첫 구성에 포함하지 않는다. 인증 정보는 전달 이미지·Git에 넣지 않는다.
