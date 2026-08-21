# Manifests

시나리오와 외부 센서 데이터의 상대 경로, checksum, 환경 조건, split을
기록한다. 실제 대용량 파일 자체는 `$AJOUNICE_DATA_ROOT`에 둔다.

Manifest는 성격에 따라 다음처럼 관리한다.

- `ros_capture_poc.json`: 기본 runner의 최소 회귀 fixture이므로 Git으로 추적한다.
- `fastbev_sunny13_1500_v1/`: 실제 학습에 사용한 frozen release split이므로
  Git으로 추적한다.
- Production·pilot·trial scenario manifest와 runtime manifest: generator가 만드는
  로컬 산출물이므로 Git으로 추적하지 않는다.

새 학습 release를 보존할 때에는 scenario 생성 manifest 전체를 넣지 않고 실제 사용한
frame의 train/val/test split, selection summary와 calibration checksum만 별도 release
디렉터리에 둔다.
