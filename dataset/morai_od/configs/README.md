# Generation configs

대량 생성기의 anchor 선택, 객체 수·거리, 날씨·시간 조합, seed와 split 정책을
YAML 설정으로 관리할 디렉터리다. 현재 프로토타입은 명령행 인자를 사용한다.

`scenario_runner_asset_catalog.yaml`은 설치된 Scenario Runner v1.7.0의 XOSC
예제에서 실제로 관찰한 MORAI 모델명을 정리한 참고 카탈로그다. MORAI 전체
Asset 목록이 아니므로 생성 전 실제 시뮬레이터에서 로드 검증해야 한다.
