# K-City Map Workflow Scripts

맵 생성·변환·검증과 대회 코스 산출에 사용한 Python 스크립트를 보관합니다.

기본 맵 경로는 저장소 루트를 기준으로 설정되어 있습니다. `autoware_*` 스크립트와 일부 버전 전환 스크립트는 추가 검증을 위해 저장소 상위의 로컬 `autoware_data/`를 선택적으로 참조합니다. Docker, Autoware 소스와 런타임 데이터는 이 저장소에 포함되지 않습니다.

새 버전 맵을 Autoware 패키지에 생성한 뒤에는 아래 명령으로 정식 보관 위치에도 반드시 발행합니다. 기본값은 v45이며, 결과는 `lanelet_map/lanelet2_map_v45.osm`입니다.

```bash
python3 scripts/kcity_map_workflow/publish_lanelet_map.py
```

다른 버전은 `--version NN`을 사용합니다. 이미 존재하는 같은 버전 파일을 의도적으로 갱신할 때만 `--force`를 추가합니다.

실행 전에 각 스크립트의 `--help`로 필요한 Python 패키지와 입·출력 경로를 확인하세요.
