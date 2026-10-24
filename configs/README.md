# 설정 진입점

현재 실행 파일은 각 분야의 README·CURRENT와 실제 launch에서 확인한다.
번호가 큰 파일이나 날짜가 최신인 실험 설정을 운영 기본값으로 간주하지 않는다.

| 분야 | 진입점 |
| --- | --- |
| 센서 | [sensors/README.md](sensors/README.md) |
| 네트워크 | [network/README.md](network/README.md) |
| 현장 MORAI 원본 | [morai_field_reference/README.md](morai_field_reference/README.md) |
| 측위 | [localization/README.md](localization/README.md) |
| 인지 | [perception/AGENTS.md](perception/AGENTS.md) |
| 데이터 수집 | [dataset/README.md](dataset/README.md) |
| 지도 자산 | [지도 README](../kcity_map/README.md) |

로컬 동시 실행·LAN 개발·대회 구성의 역할과 IP는 sensor/network 담당 문서를 따른다.
수집·실험 전용 프로필을 대회 프로필로 혼용하지 않는다.
초기 `vehicle/ioniq5.yaml`·`tf/static_tf.yaml` 예시와 `sensors/legacy/`의 옛 layout YAML은
실제 센서 JSON·launch 계약을 대체하지 않는다. 과거 설정은 분야별 `legacy/`로 분리한다.
인증정보·개인 workspace 설정·큰 지도·모델은 이 디렉터리에 추가하지 않는다.
