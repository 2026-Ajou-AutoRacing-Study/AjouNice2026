# MORAI perception dataset

이 디렉터리는 MORAI 기반 3D object detection 데이터의 시나리오 생성, 센서 수집,
검증, 변환 코드를 관리한다.

## 현재 문서 진입점

작업 전 [`docs/README.md`](docs/README.md)를 먼저 읽는다.

- Base3-Online 현재 상태: [`docs/BASE3_ONLINE_CURRENT_STATE.md`](docs/BASE3_ONLINE_CURRENT_STATE.md)
- rosbag 수집·변환 실행법: [`docs/base3_online_rosbag_collection_runbook.md`](docs/base3_online_rosbag_collection_runbook.md)
- 최신 고주로 Vehicle 수집 결과: [`docs/260919_base3_online_highway_vehicle_1k_results.md`](docs/260919_base3_online_highway_vehicle_1k_results.md)
- Base3-Static production baseline 근거: [`docs/base3_static_10k_final_capture_audit.md`](docs/base3_static_10k_final_capture_audit.md)

## 디렉터리 역할

```text
morai_od/
├── configs/       # 생성·변환 설정
├── manifests/     # scene 및 split manifest
├── metadata/      # 생성 시나리오 metadata
├── scenarios/     # MORAI scenario JSON
├── schemas/       # metadata schema
├── scripts/       # 생성·수집·audit·변환 도구
├── templates/     # scenario template
└── docs/          # CURRENT/EVIDENCE/legacy 문서
```

대용량 raw sensor와 학습 dataset은 Git 밖에 둔다. 현재 경로와 팀 전달 archive는
`docs/BASE3_ONLINE_CURRENT_STATE.md`를 기준으로 확인한다.

## Legacy

[`docs/legacy/`](docs/legacy/)에는 Base2 temporal, Sunny13/OD180, 초기 Scenario
Runner·ReplayInfo·autocapture 연구 문서를 보존한다. 해당 문서의 명령과 경로를 현재
Base3-Online 기본값으로 사용하지 않는다.
