# Base5 원격 학습·재개·백업 인덱스

현재 모델·옵션·남은 작업은 [CURRENT](CURRENT.md)만 따른다.
이 파일은 보존된 근거의 해당 절로 연결한다. `evidence/` 전체를 현재 계약으로 읽지 않는다.
정리 전 원문의 모든 explicit anchor를 유지한다. 당시 ‘E24 유지’·계획·인스턴스 상태는 현재값이 아니다.

## 원격 작업 경계

전체 30,750장 E12는 12 epoch 정상 종료했다. 완료 run을 `--resume`로 다시 시작하지 않는다.
동결 source/data/work·optimizer/scheduler/RNG·W&B identity·원격 원본을 보존한다.
현재 인스턴스 상태·요금·SSH endpoint는 실제 대상에서 확인하며 문서 날짜로 추정하지 않는다.
사용자 요청 없이 임대·학습 시작·Stop·Destroy하지 않는다. 평가 요청은 재개 승인이 아니다.
백업은 full checkpoint·metadata·COMPLETE/LATEST·전체 SHA readback으로 판정한다.
E12 최종 두 파일의 마지막 확인 상태는 중복 metadata·COMPLETE 부재로 Drive 백업 미완료다.
현재 실행기는 `scripts/runtime/start_base5_full_vast_training.sh`와 `full_data_*.py`다.
구 5K 전용 `start_base5_vast_training.sh` 및 아래 명령은 해당 동결 run에만 사용한다.
새 학습은 별도 사용자 승인과 새 계약을 확인한다. CPU 사전검사는 GPU 자격·실제 학습 완료가 아니다.

## 필요한 절 선택

<a id="next-training-cpu"></a>
- [next-training-cpu](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#next-training-cpu)

<a id="full-data-12epoch-preparation-261021"></a>
- [full-data-12epoch-preparation-261021](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-12epoch-preparation-261021)

<a id="full-data-30750-preparation-261021"></a>
- [full-data-30750-preparation-261021](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-30750-preparation-261021)

<a id="full-data-val-logging-261021"></a>
- [full-data-val-logging-261021](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-val-logging-261021)

<a id="full-data-live-training-261021"></a>
- [full-data-live-training-261021](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-live-training-261021)

<a id="full-data-speed-repair-261021"></a>
- [full-data-speed-repair-261021](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-speed-repair-261021)

<a id="full-data-e3-interrupted-evaluation-261022"></a>
- [full-data-e3-interrupted-evaluation-261022](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-e3-interrupted-evaluation-261022)

<a id="full-data-new-host-migration-261022"></a>
- [full-data-new-host-migration-261022](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-new-host-migration-261022)

<a id="full-data-e3-resumed-new-host-261022"></a>
- [full-data-e3-resumed-new-host-261022](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-e3-resumed-new-host-261022)

<a id="wandb-console-recovery-261022"></a>
- [wandb-console-recovery-261022](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#wandb-console-recovery-261022)

<a id="full-data-e5-estale-resume-261023"></a>
- [full-data-e5-estale-resume-261023](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-e5-estale-resume-261023)

<a id="full-data-e7-return-original-host-261023"></a>
- [full-data-e7-return-original-host-261023](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-e7-return-original-host-261023)

<a id="최종-사용자-실행-명령--학습-미시작"></a>
- [최종-사용자-실행-명령--학습-미시작](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최종-사용자-실행-명령--학습-미시작)

<a id="sunny15-live-training-261018"></a>
- [sunny15-live-training-261018](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny15-live-training-261018)

<a id="sunny15-cosine-complete-restarts-start-261018"></a>
- [sunny15-cosine-complete-restarts-start-261018](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny15-cosine-complete-restarts-start-261018)

<a id="sunny15-first8-distance-review-261019"></a>
- [sunny15-first8-distance-review-261019](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny15-first8-distance-review-261019)

<a id="sunny15-e8-manual-drive-backup-261019"></a>
- [sunny15-e8-manual-drive-backup-261019](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny15-e8-manual-drive-backup-261019)

<a id="sunny15-restarts-e8-result-backup-261019"></a>
- [sunny15-restarts-e8-result-backup-261019](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny15-restarts-e8-result-backup-261019)

<a id="sunny13-matched-e24-261019"></a>
- [sunny13-matched-e24-261019](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny13-matched-e24-261019)

<a id="sunny13-matched-e24-result-261019"></a>
- [sunny13-matched-e24-result-261019](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny13-matched-e24-result-261019)

<a id="base5-user-oneclick-training-261011"></a>
- [base5-user-oneclick-training-261011](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-user-oneclick-training-261011)

<a id="base5-fine-compiled-prepared-vast-261011"></a>
- [base5-fine-compiled-prepared-vast-261011](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-fine-compiled-prepared-vast-261011)

<a id="base5-fixed-image-publication-plan-261011"></a>
- [base5-fixed-image-publication-plan-261011](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-fixed-image-publication-plan-261011)

<a id="base5-vastai-4090-학습과-추론-시험-계획"></a>
- [base5-vastai-4090-학습과-추론-시험-계획](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#base5-vastai-4090-학습과-추론-시험-계획)

<a id="사용자-실행용-딸깍-학습"></a>
- [사용자-실행용-딸깍-학습](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#사용자-실행용-딸깍-학습)

<a id="최신-운영--2026-10-12-원거리-평가-확장"></a>
- [최신-운영--2026-10-12-원거리-평가-확장](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최신-운영--2026-10-12-원거리-평가-확장)

<a id="최신-실행-대상--2026-10-11-호환-nuscenes-초기화-e1-채택"></a>
- [최신-실행-대상--2026-10-11-호환-nuscenes-초기화-e1-채택](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최신-실행-대상--2026-10-11-호환-nuscenes-초기화-e1-채택)

<a id="이전-3클래스-준비-이력--2026-10-11-animal--pedestrian-통합-시점"></a>
- [이전-3클래스-준비-이력--2026-10-11-animal--pedestrian-통합-시점](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#이전-3클래스-준비-이력--2026-10-11-animal--pedestrian-통합-시점)

<a id="이전-4클래스-준비-이력--2026-10-11-2020-kst-e2-제한-검증-완료"></a>
- [이전-4클래스-준비-이력--2026-10-11-2020-kst-e2-제한-검증-완료](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#이전-4클래스-준비-이력--2026-10-11-2020-kst-e2-제한-검증-완료)

<a id="최초-준비-이력--2026-10-11-1746-kst"></a>
- [최초-준비-이력--2026-10-11-1746-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최초-준비-이력--2026-10-11-1746-kst)

<a id="시작과-관측"></a>
- [시작과-관측](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#시작과-관측)

<a id="설정과-검증-범위"></a>
- [설정과-검증-범위](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#설정과-검증-범위)

<a id="최신-fine-학습-준비--2026-10-11-1549-kst"></a>
- [최신-fine-학습-준비--2026-10-11-1549-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#최신-fine-학습-준비--2026-10-11-1549-kst)

<a id="다음-임대용-고정-이미지-게시-계획--2026-10-11-0646-kst"></a>
- [다음-임대용-고정-이미지-게시-계획--2026-10-11-0646-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#다음-임대용-고정-이미지-게시-계획--2026-10-11-0646-kst)

<a id="현재-방식과-변경할-방식"></a>
- [현재-방식과-변경할-방식](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-방식과-변경할-방식)

<a id="준비검증-순서"></a>
- [준비검증-순서](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#준비검증-순서)

<a id="2026-10-10-원격-검증-진행--본학습-시작-전"></a>
- [2026-10-10-원격-검증-진행--본학습-시작-전](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-원격-검증-진행--본학습-시작-전)

<a id="재개-후-원격-jpeg-전환-완료--2026-10-11-0108-kst"></a>
- [재개-후-원격-jpeg-전환-완료--2026-10-11-0108-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#재개-후-원격-jpeg-전환-완료--2026-10-11-0108-kst)

<a id="현재-카메라-rgb-계약과-vast-전환--2026-10-11"></a>
- [현재-카메라-rgb-계약과-vast-전환--2026-10-11](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-카메라-rgb-계약과-vast-전환--2026-10-11)

<a id="q90-100와-사전-생성-캐시--2026-10-10-2355-kst"></a>
- [q90-100와-사전-생성-캐시--2026-10-10-2355-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#q90-100와-사전-생성-캐시--2026-10-10-2355-kst)

<a id="이미지-증강-선택--2026-10-10-2305-kst-로컬-검토"></a>
- [이미지-증강-선택--2026-10-10-2305-kst-로컬-검토](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#이미지-증강-선택--2026-10-10-2305-kst-로컬-검토)

<a id="좌우-범위-선택-확정--2026-10-10-2243-kst"></a>
- [좌우-범위-선택-확정--2026-10-10-2243-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#좌우-범위-선택-확정--2026-10-10-2243-kst)

<a id="확대-검증-완료와-네트워크-gate--2026-10-10-21시"></a>
- [확대-검증-완료와-네트워크-gate--2026-10-10-21시](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#확대-검증-완료와-네트워크-gate--2026-10-10-21시)

<a id="drive-직접-다운로드-검사--2026-10-10-1718-kst"></a>
- [drive-직접-다운로드-검사--2026-10-10-1718-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#drive-직접-다운로드-검사--2026-10-10-1718-kst)

<a id="전송-방법-추가-비교--2026-10-10-1724-kst"></a>
- [전송-방법-추가-비교--2026-10-10-1724-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전송-방법-추가-비교--2026-10-10-1724-kst)

<a id="호출-제한-완화-재검사--2026-10-10-17301732-kst"></a>
- [호출-제한-완화-재검사--2026-10-10-17301732-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#호출-제한-완화-재검사--2026-10-10-17301732-kst)

<a id="전용-drive-앱-인증과-속도-재검사--2026-10-10-1757-kst"></a>
- [전용-drive-앱-인증과-속도-재검사--2026-10-10-1757-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전용-drive-앱-인증과-속도-재검사--2026-10-10-1757-kst)

<a id="전용-연결-8병렬-검사--2026-10-10-1804-kst"></a>
- [전용-연결-8병렬-검사--2026-10-10-1804-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#전용-연결-8병렬-검사--2026-10-10-1804-kst)

<a id="두-번째-호스트의-제한-검사--2026-10-10-1817-kst"></a>
- [두-번째-호스트의-제한-검사--2026-10-10-1817-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#두-번째-호스트의-제한-검사--2026-10-10-1817-kst)

<a id="수동-전송-중단-원인과-부족-조각-복구--2026-10-10-19131932-kst"></a>
- [수동-전송-중단-원인과-부족-조각-복구--2026-10-10-19131932-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#수동-전송-중단-원인과-부족-조각-복구--2026-10-10-19131932-kst)

<a id="부족-조각-복구와-전체-데이터-검증-완료--2026-10-10-1949-kst"></a>
- [부족-조각-복구와-전체-데이터-검증-완료--2026-10-10-1949-kst](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#부족-조각-복구와-전체-데이터-검증-완료--2026-10-10-1949-kst)

<a id="현재-전송-유지와-cloud-sync-후속-선택--2026-10-10-합의"></a>
- [현재-전송-유지와-cloud-sync-후속-선택--2026-10-10-합의](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#현재-전송-유지와-cloud-sync-후속-선택--2026-10-10-합의)

<a id="초기-준비의-역할-분담--아직-수행하지-않았다"></a>
- [초기-준비의-역할-분담--아직-수행하지-않았다](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#초기-준비의-역할-분담--아직-수행하지-않았다)

<a id="2026-10-10-vast-템플릿-안내와-블로그용-증거-기록"></a>
- [2026-10-10-vast-템플릿-안내와-블로그용-증거-기록](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-vast-템플릿-안내와-블로그용-증거-기록)

<a id="화면에서-관측한-내용과-선택-이유"></a>
- [화면에서-관측한-내용과-선택-이유](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#화면에서-관측한-내용과-선택-이유)

<a id="안내한-입력값--적용-확인-전"></a>
- [안내한-입력값--적용-확인-전](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#안내한-입력값--적용-확인-전)

<a id="학습-gpu와-배치-선택-논의--실측-전"></a>
- [학습-gpu와-배치-선택-논의--실측-전](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#학습-gpu와-배치-선택-논의--실측-전)

<a id="캡처-원본-보관"></a>
- [캡처-원본-보관](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#캡처-원본-보관)

<a id="gpu-후보-추천--2026-10-10-161856-kst-캡처-기준"></a>
- [gpu-후보-추천--2026-10-10-161856-kst-캡처-기준](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#gpu-후보-추천--2026-10-10-161856-kst-캡처-기준)

<a id="2026-10-10-sunny13-5k-drive-원본-백업"></a>
- [2026-10-10-sunny13-5k-drive-원본-백업](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-sunny13-5k-drive-원본-백업)

<a id="vast-전송-방식-선택"></a>
- [vast-전송-방식-선택](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#vast-전송-방식-선택)

<a id="2026-10-10-로컬-준비-결과"></a>
- [2026-10-10-로컬-준비-결과](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026-10-10-로컬-준비-결과)

<a id="임대한-환경에서의-순서"></a>
- [임대한-환경에서의-순서](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#임대한-환경에서의-순서)

<a id="로컬-정리-결과와-보존-원칙"></a>
- [로컬-정리-결과와-보존-원칙](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#로컬-정리-결과와-보존-원칙)

<a id="임대-사양-추천"></a>
- [임대-사양-추천](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#임대-사양-추천)

<a id="2026년10월9일-가격-관측"></a>
- [2026년10월9일-가격-관측](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#2026년10월9일-가격-관측)

<a id="저장공간과-업로드-범위"></a>
- [저장공간과-업로드-범위](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#저장공간과-업로드-범위)

<a id="ssh와-실행-환경"></a>
- [ssh와-실행-환경](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#ssh와-실행-환경)

<a id="로컬에서-원격-학습까지"></a>
- [로컬에서-원격-학습까지](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#로컬에서-원격-학습까지)

<a id="rosbag-dev-val과-최종-평가"></a>
- [rosbag-dev-val과-최종-평가](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#rosbag-dev-val과-최종-평가)

<a id="중단-복구와-종료-기준"></a>
- [중단-복구와-종료-기준](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#중단-복구와-종료-기준)

<a id="원격-morai는-학습-환경과-별도-검토"></a>
- [원격-morai는-학습-환경과-별도-검토](evidence/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#원격-morai는-학습-환경과-별도-검토)
