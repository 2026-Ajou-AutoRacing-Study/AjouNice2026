# Base5 원격 실행과 보존 안내

문서 상태: RUNBOOK + COMPATIBILITY ROUTER. 확인 기준: 2026-10-15.
Sunny13 24 epoch 학습·E9/E24 외부 백업은 완료했다. 현재 모델·다음 작업은 [CURRENT](CURRENT.md)를 따른다.
아래 기존 명령은 완료된 동결 run의 점검/복구용이며 새로운 학습 준비를 자동으로 의미하지 않는다.

## 점검과 재개 경계

- 실행 진입점은 [start_base5_vast_training.sh](../../scripts/runtime/start_base5_vast_training.sh)다.
- 인자 없는 실행은 원격 run을 시작할 수 있다. 문서 확인만을 위해 실행하지 않는다.
- 사용자가 원격 점검을 요청했을 때 `bash scripts/runtime/start_base5_vast_training.sh --check --resume`로 계약·연결·W&B·Drive를 확인한다.
- 같은 동결 run의 미완료 에폭을 재개할 때만 사용자 승인 후 `--resume`를 사용한다.
- 새 환경·클래스·초기화·스케줄 변경은 새 source/data/work 계약을 만든다. 완료 run의 hash를 덮어쓰지 않는다.
- W&B run identity·optimizer/scheduler/RNG·완료 epoch 재개 범위를 보존한다. 정확한 mid-epoch replay를 보장하지 않는다.
- 백업은 업로드 폴더 존재가 아니라 full checkpoint·metadata·COMPLETE/LATEST·SHA readback으로 확인한다.
- GPU Stop 후 재할당과 재시작 불가가 가능하다. 데이터 복사·백업 확인 없이 Destroy하지 않는다.
- SSH endpoint·현재 요금·가용 GPU는 실제 인스턴스에서 확인한다. 이력의 IP·가격을 현재값으로 쓰지 않는다.
- 새 임대용 고정 팀 이미지의 registry 게시/pull 검증은 별도 미완료 작업이다.

<a id="next-training-cpu"></a>

## 다음 학습의 CPU 사전검사

2026-10-15 사용자 승인으로 **새 학습만** compile OFF로 준비했다. 완료 E24의 학습 ON 설정,
동결 Vast 실행기·optimizer·W&B run·가중치·resume 계약은 수정하지 않는다.
OFF가 정확도를 높인다는 인증이나 기존 E24가 잘못 학습됐다는 뜻은 아니다.

- 설정: `bevfusion_base5_fine_512_depth25_next_training.py`. 기존 nearest runtime을 상속하며 compile OFF를 명시한다.
- train/val 공통 nearest_positive/sorted, Q90 JPEG, upright 7D·Animal→Pedestrian, 512×1056·5cm/20cm·depth 0.25m를 유지한다.
- SaveSensorData 강도는 원래 0~1이며 학습에서 ÷255하지 않는다. ROS 0~255 변환은 추론 입력에만 적용한다.
- 새 실행기는 SSH·GPU·모델 registry·학습을 시작하지 않는다. `--run`·`--resume`를 받지 않으며 새 빈 work-dir만 허용한다.

기존 BEVFusion Python 환경 안에서 실행한다. 호스트 시스템 Python에는 MMEngine이 없을 수 있으며,
사전검사를 위해 의존성을 재설치하거나 GPU를 빌릴 필요는 없다. 기존 팀 Docker를 GPU 없이 사용해도 된다.

```bash
bash scripts/runtime/prepare_base5_next_training.sh --check-config

bash scripts/runtime/prepare_base5_next_training.sh \
  --data "$BASE5_DATA" \
  --init-checkpoint "$BASE5_INITIALIZER" \
  --work-dir "$PWD/artifacts/training/base5_next_cpu_preflight" \
  --verify-payload-hashes --sample-clouds 16 --wandb
```

`BASE5_DATA`는 신뢰하는 3클래스 native-JPEG portable root, `BASE5_INITIALIZER`는 별도로 검증한
호환 초기화 파일 경로다. CPU 단계에서는 초기화 파일의 SHA만 기록하며 state shape를 검증하지 않는다.
기본 24 epoch·batch 4·누적 1·lr 1e-4는 준비 계획 값이고 최종 30K 학습 예산의 확정이 아니다.

전체 GT·split·입력 경로와 선택 시 전체 payload 해시를 검사하고, LiDAR 강도는 split마다 고르게
뽑은 표본으로 확인한다. `checked_model_config.py`와 `next_training_preflight.json`만 생성한다.
후자는 실제 direct runner의 `run_manifest.json`이나 GPU/패키지 자격 검증을 대체하지 않으며 이를 복사해 E24에 강제 resume하지 않는다.
계획에 기록된 후속 prepare 명령은 기존 거리별 AP/W&B 진단 확장을 유지하지만 `--run`은 포함하지 않는다.
실제 학습은 대상 GPU에서 초기화 strict load·forward/backward·비용·재개·W&B/Drive 백업을 검증하고
새 source/data/work 계약을 동결한 뒤 별도로 승인한다. CPU artifact의 컨테이너 경로는 새 호스트에서 다시 준비한다.
[2026-10-15 CPU 검증 근거](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-next-training-cpu-261015)는 실제 5K·GT 전체와 강도 표본 검사의 범위를 구분한다.

## 상세 운영과 실측의 보존

설치·전송·OAuth·백업·학습 준비·재개 실험의 명령은 아래 해당 anchor의 당시 기록으로 연결한다.
문서 개편은 원격 접속·이미지 게시·학습·과금 상태를 변경하지 않았다.

## 기존 링크 호환

<a id="base5-user-oneclick-training-261011"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-user-oneclick-training-261011).

<a id="base5-fine-compiled-prepared-vast-261011"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-fine-compiled-prepared-vast-261011).

<a id="base5-fixed-image-publication-plan-261011"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-fixed-image-publication-plan-261011).

<a id="base5-vastai-4090-학습과-추론-시험-계획"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-vastai-4090-학습과-추론-시험-계획).

<a id="사용자-실행용-딸깍-학습"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#사용자-실행용-딸깍-학습).

<a id="최신-운영--2026-10-12-원거리-평가-확장"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최신-운영--2026-10-12-원거리-평가-확장).

<a id="최신-실행-대상--2026-10-11-호환-nuscenes-초기화-e1-채택"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최신-실행-대상--2026-10-11-호환-nuscenes-초기화-e1-채택).

<a id="이전-3클래스-준비-이력--2026-10-11-animal--pedestrian-통합-시점"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#이전-3클래스-준비-이력--2026-10-11-animal--pedestrian-통합-시점).

<a id="이전-4클래스-준비-이력--2026-10-11-2020-kst-e2-제한-검증-완료"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#이전-4클래스-준비-이력--2026-10-11-2020-kst-e2-제한-검증-완료).

<a id="최초-준비-이력--2026-10-11-1746-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최초-준비-이력--2026-10-11-1746-kst).

<a id="시작과-관측"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#시작과-관측).

<a id="설정과-검증-범위"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#설정과-검증-범위).

<a id="최신-fine-학습-준비--2026-10-11-1549-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최신-fine-학습-준비--2026-10-11-1549-kst).

<a id="다음-임대용-고정-이미지-게시-계획--2026-10-11-0646-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#다음-임대용-고정-이미지-게시-계획--2026-10-11-0646-kst).

<a id="현재-방식과-변경할-방식"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-방식과-변경할-방식).

<a id="준비검증-순서"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#준비검증-순서).

<a id="2026-10-10-원격-검증-진행--본학습-시작-전"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-원격-검증-진행--본학습-시작-전).

<a id="재개-후-원격-jpeg-전환-완료--2026-10-11-0108-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#재개-후-원격-jpeg-전환-완료--2026-10-11-0108-kst).

<a id="현재-카메라-rgb-계약과-vast-전환--2026-10-11"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-카메라-rgb-계약과-vast-전환--2026-10-11).

<a id="q90-100와-사전-생성-캐시--2026-10-10-2355-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#q90-100와-사전-생성-캐시--2026-10-10-2355-kst).

<a id="이미지-증강-선택--2026-10-10-2305-kst-로컬-검토"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#이미지-증강-선택--2026-10-10-2305-kst-로컬-검토).

<a id="좌우-범위-선택-확정--2026-10-10-2243-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#좌우-범위-선택-확정--2026-10-10-2243-kst).

<a id="확대-검증-완료와-네트워크-gate--2026-10-10-21시"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#확대-검증-완료와-네트워크-gate--2026-10-10-21시).

<a id="drive-직접-다운로드-검사--2026-10-10-1718-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#drive-직접-다운로드-검사--2026-10-10-1718-kst).

<a id="전송-방법-추가-비교--2026-10-10-1724-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전송-방법-추가-비교--2026-10-10-1724-kst).

<a id="호출-제한-완화-재검사--2026-10-10-17301732-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#호출-제한-완화-재검사--2026-10-10-17301732-kst).

<a id="전용-drive-앱-인증과-속도-재검사--2026-10-10-1757-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전용-drive-앱-인증과-속도-재검사--2026-10-10-1757-kst).

<a id="전용-연결-8병렬-검사--2026-10-10-1804-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전용-연결-8병렬-검사--2026-10-10-1804-kst).

<a id="두-번째-호스트의-제한-검사--2026-10-10-1817-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#두-번째-호스트의-제한-검사--2026-10-10-1817-kst).

<a id="수동-전송-중단-원인과-부족-조각-복구--2026-10-10-19131932-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#수동-전송-중단-원인과-부족-조각-복구--2026-10-10-19131932-kst).

<a id="부족-조각-복구와-전체-데이터-검증-완료--2026-10-10-1949-kst"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#부족-조각-복구와-전체-데이터-검증-완료--2026-10-10-1949-kst).

<a id="현재-전송-유지와-cloud-sync-후속-선택--2026-10-10-합의"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-전송-유지와-cloud-sync-후속-선택--2026-10-10-합의).

<a id="초기-준비의-역할-분담--아직-수행하지-않았다"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#초기-준비의-역할-분담--아직-수행하지-않았다).

<a id="2026-10-10-vast-템플릿-안내와-블로그용-증거-기록"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-vast-템플릿-안내와-블로그용-증거-기록).

<a id="화면에서-관측한-내용과-선택-이유"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#화면에서-관측한-내용과-선택-이유).

<a id="안내한-입력값--적용-확인-전"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#안내한-입력값--적용-확인-전).

<a id="학습-gpu와-배치-선택-논의--실측-전"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#학습-gpu와-배치-선택-논의--실측-전).

<a id="캡처-원본-보관"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#캡처-원본-보관).

<a id="gpu-후보-추천--2026-10-10-161856-kst-캡처-기준"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#gpu-후보-추천--2026-10-10-161856-kst-캡처-기준).

<a id="2026-10-10-sunny13-5k-drive-원본-백업"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-sunny13-5k-drive-원본-백업).

<a id="vast-전송-방식-선택"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#vast-전송-방식-선택).

<a id="2026-10-10-로컬-준비-결과"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-로컬-준비-결과).

<a id="임대한-환경에서의-순서"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#임대한-환경에서의-순서).

<a id="로컬-정리-결과와-보존-원칙"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#로컬-정리-결과와-보존-원칙).

<a id="임대-사양-추천"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#임대-사양-추천).

<a id="2026년10월9일-가격-관측"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026년10월9일-가격-관측).

<a id="저장공간과-업로드-범위"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#저장공간과-업로드-범위).

<a id="ssh와-실행-환경"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#ssh와-실행-환경).

<a id="로컬에서-원격-학습까지"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#로컬에서-원격-학습까지).

<a id="rosbag-dev-val과-최종-평가"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#rosbag-dev-val과-최종-평가).

<a id="중단-복구와-종료-기준"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#중단-복구와-종료-기준).

<a id="원격-morai는-학습-환경과-별도-검토"></a>
당시 이력: [원문 절](../perception/legacy/context_261012/docs__bevfusion__261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#원격-morai는-학습-환경과-별도-검토).
