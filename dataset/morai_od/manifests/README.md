# Manifests

시나리오와 외부 센서 데이터의 상대 경로, checksum, 환경 조건, split을
기록한다. 실제 대용량 파일 자체는 `$AJOUNICE_DATA_ROOT`에 둔다.

OD180 파일럿은 `od180_base.json`과 환경별 manifest 6개를 사용한다.
`od180_runs.jsonl`은 30개 base와 여섯 환경의 180 run matrix이고,
`od180_train.txt`, `od180_val.txt`, `od180_test.txt`는 환경과 무관한
base scenario split이다.
